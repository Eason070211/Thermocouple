#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NIST ITS-90 分度表 -> TC_TABLE v1 转换 + 校验工具

数据来源
--------
NIST Standard Reference Database 60 (SRD 60), "Thermocouple Data":
    https://its90.nist.gov/ThermoDownloads
    https://its90.nist.gov/downloadFiles/type_<x>.tab.txt

那个 .tab.txt 文件是**纯文本**, 里面同时包含:
    1) 逐度分度表 (温度 -> 热电势 mV, 1 °C 步长)
    2) 参考函数(正)与近似逆函数的系数

本工具做四件事:
    1) 下载 (带本地缓存, 可 --offline 复用)
    2) 解析成 (温度 °C, 热电势 µV) 点列
    3) 写出工程内统一格式 tables/tc_type_<x>.csv  (TC_TABLE v1)
    4) 校验 —— 这一步才是重点, 见 check_* 系列函数:
         a) 温度严格递增、无重复冲突
         b) 电势严格单调递增
         c) 表 vs 文件自带的正多项式: 逐点比对 (抓解析错位)
         d) K 型额外锚点 + 与固件 thermocouple.c 的独立实现比对
       任何一项不过 -> 脚本以非 0 退出, 绝不产出可疑的表。

用法
----
    python tools/nist_tc_tables.py                # K/J/T/E/N 全部重建
    python tools/nist_tc_tables.py --types K      # 只重建 K
    python tools/nist_tc_tables.py --offline      # 只用缓存
    python tools/nist_tc_tables.py --check-only   # 只校验已生成的表
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
import urllib.request
from datetime import datetime, timezone
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 路径与常量
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)
CACHE_DIR = os.path.join(HERE, "_nist_cache")
TABLE_DIR = os.path.join(PROJECT_ROOT, "tables")

BASE_URL = "https://its90.nist.gov/downloadFiles/type_%s.tab.txt"

#: 本次落地的 5 种分度号; K 是默认(也是固件唯一支持的)
TYPES: Tuple[str, ...] = ("K", "J", "T", "E", "N")
DEFAULT_TYPE = "K"

#: 每种型号的覆盖范围 (°C), 来自 NIST 下载页的说明
TYPE_RANGE: Dict[str, Tuple[int, int]] = {
    "K": (-270, 1372),
    "J": (-210, 1200),
    "T": (-270, 400),
    "E": (-270, 1000),
    "N": (-270, 1300),
}

STANDARD = "ITS-90 (NIST SRD 60) / IEC 60584-1 / ASTM E230 / GB/T 16839"

#: K 型锚点: 温度 °C -> 热电势 µV。这些值与固件 thermocouple.c 的自检表一致,
#: 是**独立于本文件解析结果**的硬编码期望值, 用来确认"下来的文件本身是对的"。
K_ANCHORS: Tuple[Tuple[float, float], ...] = (
    (0.0, 0.0),
    (25.0, 1000.24),
    (500.0, 20644.29),
    (1000.0, 41275.61),
    (1372.0, 54886.36),
)
ANCHOR_TOL_UV = 3.0        # 表只保留到 0.001 mV = 1 µV, 留 3 µV 余量
POLY_TOL_UV = 2.0          # 表 vs 文件自带参考函数的最大允许偏差


# ---------------------------------------------------------------------------
# 1) 下载 / 缓存
# ---------------------------------------------------------------------------
def download(types: Sequence[str], offline: bool) -> Dict[str, str]:
    """下载 (或复用缓存) .tab.txt, 返回 {类型: 本地路径}。"""
    os.makedirs(CACHE_DIR, exist_ok=True)
    paths: Dict[str, str] = {}
    for tc in types:
        local = os.path.join(CACHE_DIR, "type_%s.tab.txt" % tc.lower())
        if os.path.exists(local) and os.path.getsize(local) > 2000:
            print("  [cache] %s" % os.path.basename(local))
            paths[tc] = local
            continue
        if offline:
            raise SystemExit("缺少缓存且指定了 --offline: %s" % local)
        url = BASE_URL % tc.lower()
        print("  [fetch] %s" % url)
        req = urllib.request.Request(url, headers={"User-Agent": "tc-table-builder/1.0"})
        with urllib.request.urlopen(req, timeout=30) as resp:
            data = resp.read()
        if len(data) < 2000:
            raise SystemExit("下载内容过短, 疑似失败: %s" % url)
        with open(local, "wb") as fh:
            fh.write(data)
        paths[tc] = local
    return paths


def read_text(path: str) -> str:
    """NIST 文件里 ° 是单字节 0xB0 (latin-1), 不是 UTF-8。"""
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8", "cp1252", "latin-1"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1", errors="replace")


# ---------------------------------------------------------------------------
# 2) 解析
# ---------------------------------------------------------------------------
_COL_HEADER_RE = re.compile(r"^\s*\S+\s+((?:-?\d+\s+){4,}-?\d+)\s*$")
_DATA_RE = re.compile(r"^\s*(-?\d+)\s+((?:-?\d+\.\d+)(?:\s+-?\d+\.\d+)*)\s*$")


def parse_grid(text: str) -> List[Tuple[int, float]]:
    """解析逐度表 -> [(温度 °C, 热电势 µV), ...], 已排序去重。

    文件结构是"带列偏移的矩阵": 每行给出基准温度 + 11 个偏移列 (0,-1..-10 或 0..10),
    所以必须先记住最近一个列头, 才能把每个数还原成绝对温度。
    """
    points: Dict[int, float] = {}
    conflicts: List[str] = []
    cols: Optional[List[int]] = None

    for lineno, line in enumerate(text.splitlines(), 1):
        m = _COL_HEADER_RE.match(line)
        if m:
            cand = [int(x) for x in m.group(1).split()]
            # 只认"0,1,2,...,10"或"0,-1,...,-10"这种列头
            if cand[0] == 0 and len(cand) >= 5:
                cols = cand
            continue

        m = _DATA_RE.match(line)
        if not m or cols is None:
            continue
        base = int(m.group(1))
        values = [float(v) for v in m.group(2).split()]
        if len(values) > len(cols):
            conflicts.append("第 %d 行数值个数(%d)超过列头(%d)" % (lineno, len(values), len(cols)))
            continue
        for offset, mv in zip(cols, values):
            temp = base + offset
            uv = mv * 1000.0
            if temp in points and abs(points[temp] - uv) > 0.5:
                conflicts.append("温度 %d 出现两个不一致的值: %.3f / %.3f µV"
                                 % (temp, points[temp], uv))
            points[temp] = uv

    if conflicts:
        raise SystemExit("解析冲突:\n  " + "\n  ".join(conflicts[:10]))

    return sorted(points.items())


def parse_reference_function(text: str) -> Optional[Dict]:
    """解析文件自带的正(参考)函数系数: E(mV) = Σ c_i t^i (+ a0·exp(a1(t-a2)²))。"""
    lines = text.splitlines()
    ranges: List[Tuple[float, float, List[float]]] = []
    expo: Dict[str, float] = {}
    i = 0
    in_ref = False
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("name:") and "reference function" in line:
            in_ref = True
        elif line.startswith("name:"):
            in_ref = False
        elif in_ref and line.startswith("range:"):
            parts = [p.strip() for p in line.split(":", 1)[1].split(",")]
            try:
                lo, hi, order = float(parts[0]), float(parts[1]), int(parts[2])
            except (ValueError, IndexError):
                i += 1
                continue
            # ★ range 行的第 3 个字段是**阶数**(最高次幂), 所以要读 order+1 个系数。
            #   例: K 型 "range: -270.000, 0.000, 10" -> 11 个系数 (与固件
            #   thermocouple.c 的 TC_K_C_LO[11] 一致)。
            count = order + 1
            coefs: List[float] = []
            j = i + 1
            while j < len(lines) and len(coefs) < count:
                token = lines[j].strip()
                if token:
                    try:
                        coefs.append(float(token))
                    except ValueError:
                        break
                j += 1
            if len(coefs) == count:
                ranges.append((lo, hi, coefs))
            i = j
            continue
        elif in_ref and line.startswith("a0"):
            expo["a0"] = float(line.split("=")[1])
        elif in_ref and line.startswith("a1"):
            expo["a1"] = float(line.split("=")[1])
        elif in_ref and line.startswith("a2"):
            expo["a2"] = float(line.split("=")[1])
        i += 1

    if not ranges:
        return None
    return {"ranges": ranges, "expo": expo}


def poly_emf_uv(ref: Dict, temp_c: float) -> Optional[float]:
    """用解析出的参考函数算 E(µV); 超出覆盖范围返回 None。

    ★ 注意 t=0 的处理: 0 °C 同时落在"负温段(-270,0]"和"正温段[0,1372)"里。
      负温段的多项式在 t=0 处给出 0, 而**指数修正项只属于正温段**
      (文件里写明 "The equation above 0 °C is of the form ... + a0 exp(...)")。
      正温段在 t=0 处: 多项式给 -17.6 µV, 指数项补回 +17.6 µV, 正好为 0 ——
      这正是该指数项存在的意义。所以 t>=0 时必须选正温段, 否则 t=0 会差 17.6 µV。
    """
    chosen: Optional[Tuple[float, float, List[float]]] = None
    for lo, hi, coefs in ref["ranges"]:
        if lo <= temp_c <= hi:
            if temp_c >= 0.0 and lo < 0.0:
                continue                      # 正温点不要用负温段
            chosen = (lo, hi, coefs)
            break
    if chosen is None:
        return None

    lo, hi, coefs = chosen
    acc = 0.0
    for c in reversed(coefs):
        acc = acc * temp_c + c
    mv = acc
    expo = ref.get("expo") or {}
    if temp_c >= 0.0 and len(expo) == 3:
        d = temp_c - expo["a2"]
        mv += expo["a0"] * math.exp(expo["a1"] * d * d)
    return mv * 1000.0


# ---------------------------------------------------------------------------
# 3) 校验
# ---------------------------------------------------------------------------
def check_monotonic(points: Sequence[Tuple[int, float]], tc: str) -> List[str]:
    """温度严格递增; 电势只要求**非递减**。

    ★ 为什么电势不能要求严格递增: NIST 表只保留到 0.001 mV (1 µV), 而 N 型在
      -270 °C 附近热电率极小 (约 0.3 µV/°C), 四舍五入后会出现相邻点数值相等。
      这是表的分辨率限制, 不是数据错误 —— 它对应的温度不确定度 < 0.03 °C。
      相等的段落在求逆时要特殊处理 (见 pc_ui/tc_table.py)。
    """
    errors: List[str] = []
    flats = 0
    for (t0, e0), (t1, e1) in zip(points, points[1:]):
        if t1 <= t0:
            errors.append("%s: 温度未严格递增 %d -> %d" % (tc, t0, t1))
        if e1 < e0:
            errors.append("%s: 电势下降 t=%d: %.1f -> %.1f µV" % (tc, t1, e0, e1))
        elif e1 == e0:
            flats += 1
    if flats:
        print("      注: 有 %d 处相邻电势相等 (表分辨率 1 µV 导致, 正常)" % flats)
    return errors


def check_vs_reference(points: Sequence[Tuple[int, float]], ref: Optional[Dict],
                       tc: str) -> Tuple[List[str], float, int]:
    """表 vs 文件自带参考函数逐点比对。返回 (错误, 最大偏差µV, 比对点数)。"""
    if ref is None:
        return (["%s: 未解析到参考函数系数" % tc], 0.0, 0)
    errors: List[str] = []
    max_dev = 0.0
    compared = 0
    for temp, uv in points:
        ref_uv = poly_emf_uv(ref, float(temp))
        if ref_uv is None:
            continue
        dev = abs(ref_uv - uv)
        compared += 1
        max_dev = max(max_dev, dev)
        if dev > POLY_TOL_UV:
            errors.append("%s: t=%d 表 %.1f µV vs 参考函数 %.1f µV (差 %.1f µV)"
                          % (tc, temp, uv, ref_uv, dev))
    if compared < 100:
        errors.append("%s: 只比对了 %d 个点, 太少" % (tc, compared))
    return (errors[:8], max_dev, compared)


def check_anchors(points: Sequence[Tuple[int, float]], tc: str) -> List[str]:
    """K 型锚点硬核对 (独立于解析路径的期望值)。"""
    if tc != "K":
        return []
    table = dict(points)
    errors: List[str] = []
    for temp, expect in K_ANCHORS:
        key = int(round(temp))
        if key not in table:
            errors.append("K: 锚点 %g °C 不在表中" % temp)
            continue
        got = table[key]
        if abs(got - expect) > ANCHOR_TOL_UV:
            errors.append("K: 锚点 %g °C 期望 %.2f µV, 实得 %.2f µV" % (temp, expect, got))
    return errors


def check_vs_firmware_K(points: Sequence[Tuple[int, float]]) -> Tuple[List[str], float]:
    """K 型: 与固件 Core/Src/thermocouple.c 的系数(独立实现)比对。

    固件用的是同一份 NIST 系数, 但代码路径完全独立 (C 里的 Horner + 指数项),
    所以这条能同时抓"表解析错"和"固件系数抄错"。
    """
    C_LO = (0.000000000000E+00, 0.394501280250E-01, 0.236223735980E-04,
            -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10,
            -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16,
            -0.198892668780E-19, -0.163226974860E-22)
    C_HI = (-0.176004136860E-01, 0.389212049750E-01, 0.185587700320E-04,
            -0.994575928740E-07, 0.318409457190E-09, -0.560728448890E-12,
            0.560750590590E-15, -0.320207200030E-18, 0.971511471520E-22,
            -0.121047212750E-25)
    A0, A1, A2 = 0.118597600000E+00, -0.118343200000E-03, 0.126968600000E+03

    def emf_uv(temp: float) -> float:
        coefs = C_LO if temp < 0.0 else C_HI
        acc = 0.0
        for c in reversed(coefs):
            acc = acc * temp + c
        mv = acc
        if temp >= 0.0:
            d = temp - A2
            mv += A0 * math.exp(A1 * d * d)
        return mv * 1000.0

    errors: List[str] = []
    max_dev = 0.0
    for temp, uv in points:
        if not (-270 <= temp <= 1372):
            continue
        dev = abs(emf_uv(float(temp)) - uv)
        max_dev = max(max_dev, dev)
        if dev > POLY_TOL_UV:
            errors.append("K: t=%d 表 %.1f µV vs 固件系数 %.1f µV (差 %.1f µV)"
                          % (temp, uv, emf_uv(float(temp)), dev))
    return (errors[:8], max_dev)


# ---------------------------------------------------------------------------
# 4) 写出 TC_TABLE v1
# ---------------------------------------------------------------------------
def build_table_file(tc: str, points: Sequence[Tuple[int, float]],
                     source: str, max_dev: float) -> Tuple[str, str]:
    """返回 (文件文本, 数据体 sha256)。"""
    lo, hi = TYPE_RANGE[tc]
    body_lines = ["%d,%.1f" % (t, uv) for t, uv in points]
    body = "\n".join(body_lines) + "\n"
    digest = hashlib.sha256(body.encode("utf-8")).hexdigest()

    header = [
        "# TC_TABLE v1",
        "# 本文件由 tools/nist_tc_tables.py 自动生成, 请勿手工编辑",
        "#",
        "# type       = %s" % tc,
        "# direction  = T2E            (温度 -> 热电势)",
        "# ref_temp_C = 0              (参考端 0 °C; 冷端补偿由软件负责)",
        "# temp_unit  = C",
        "# emf_unit   = uV             (★ 单位必须显式声明, 见 README)",
        "# temp_min   = %d" % lo,
        "# temp_max   = %d" % hi,
        "# step_C     = 1",
        "# points     = %d" % len(points),
        "# standard   = %s" % STANDARD,
        "# source     = %s" % source,
        "# max_dev_vs_reference_uV = %.3f" % max_dev,
        "# created    = %s" % datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "# sha256     = %s" % digest,
        "temp_C,emf_uV",
    ]
    return ("\n".join(header) + "\n" + body, digest)


def parse_header(path: str) -> Tuple[Dict[str, str], str]:
    """读回 TC_TABLE v1: 返回 (元数据字典, 数据体文本)。"""
    meta: Dict[str, str] = {}
    with open(path, encoding="utf-8") as fh:
        lines = fh.read().splitlines()
    body_start = None
    for idx, line in enumerate(lines):
        if line.startswith("#"):
            item = line[1:].strip()
            if "=" in item and not item.startswith("TC_TABLE"):
                key, _, val = item.partition("=")
                meta[key.strip()] = val.split("(")[0].strip()
        elif line.startswith("temp_C,"):
            body_start = idx + 1
            break
    if body_start is None:
        raise SystemExit("%s: 不是合法的 TC_TABLE v1 (缺少表头)" % path)
    return meta, "\n".join(lines[body_start:]) + "\n"


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="NIST ITS-90 -> TC_TABLE v1 转换与校验")
    ap.add_argument("--types", default=",".join(TYPES),
                    help="逗号分隔的分度号, 默认 %s" % ",".join(TYPES))
    ap.add_argument("--offline", action="store_true", help="只用本地缓存")
    ap.add_argument("--check-only", action="store_true", help="只校验已生成的表")
    args = ap.parse_args(argv)

    types = [t.strip().upper() for t in args.types.split(",") if t.strip()]
    unk = [t for t in types if t not in TYPE_RANGE]
    if unk:
        raise SystemExit("不支持的分度号: %s (可选 %s)" % (unk, list(TYPE_RANGE)))

    os.makedirs(TABLE_DIR, exist_ok=True)
    print("=" * 74)
    print("NIST ITS-90 分度表 -> TC_TABLE v1   (默认分度号: %s)" % DEFAULT_TYPE)
    print("=" * 74)

    all_errors: List[str] = []
    manifest: Dict[str, dict] = {}

    if not args.check_only:
        print("\n[1/4] 获取源文件")
        paths = download(types, args.offline)

        for tc in types:
            print("\n[2/4] 解析 %s 型" % tc)
            text = read_text(paths[tc])
            points = parse_grid(text)
            ref = parse_reference_function(text)
            print("      表点 %d 个, 温度 %d…%d °C" % (len(points), points[0][0], points[-1][0]))
            print("      参考函数: %s" % ("已解析 (%d 段)" % len(ref["ranges"]) if ref else "无"))

            print("[3/4] 校验 %s 型" % tc)
            errs: List[str] = []
            errs += check_monotonic(points, tc)
            ref_errs, max_dev, compared = check_vs_reference(points, ref, tc)
            errs += ref_errs
            errs += check_anchors(points, tc)
            fw_dev = 0.0
            if tc == "K":
                fw_errs, fw_dev = check_vs_firmware_K(points)
                errs += fw_errs

            if errs:
                all_errors += errs
                print("      [FAIL]")
                for e in errs:
                    print("        - %s" % e)
                continue
            print("      [OK] 单调 + 参考函数比对 %d 点 (最大偏差 %.3f µV)" % (compared, max_dev))
            if tc == "K":
                print("      [OK] K 型锚点 %d 个 + 固件系数比对 (最大偏差 %.3f µV)"
                      % (len(K_ANCHORS), fw_dev))

            out_path = os.path.join(TABLE_DIR, "tc_type_%s.csv" % tc.lower())
            text_out, digest = build_table_file(
                tc, points,
                "NIST SRD 60 %s (https://its90.nist.gov/downloadFiles/type_%s.tab.txt)"
                % (os.path.basename(paths[tc]), tc.lower()),
                max_dev)
            with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(text_out)
            size = os.path.getsize(out_path)
            print("[4/4] 写出 %s (%d 字节, sha256 %s…)"
                  % (os.path.relpath(out_path, PROJECT_ROOT), size, digest[:12]))
            manifest[tc] = {
                "file": os.path.basename(out_path),
                "type": tc,
                "points": len(points),
                "temp_min": points[0][0],
                "temp_max": points[-1][0],
                "emf_min_uV": round(points[0][1], 1),
                "emf_max_uV": round(points[-1][1], 1),
                "sha256": digest,
                "max_dev_vs_reference_uV": round(max_dev, 3),
                "max_dev_vs_firmware_uV": round(fw_dev, 3) if tc == "K" else None,
                "standard": STANDARD,
            }

    # ---- 重新读回磁盘上的表再校验一遍 (check-only 模式也走这条路) ----
    print("\n[复核] 读回磁盘上的表文件并重新校验")
    for tc in types:
        path = os.path.join(TABLE_DIR, "tc_type_%s.csv" % tc.lower())
        if not os.path.exists(path):
            all_errors.append("%s: 缺少表文件 %s" % (tc, path))
            continue
        meta, body = parse_header(path)
        digest = hashlib.sha256(body.encode("utf-8")).hexdigest()
        if meta.get("sha256") != digest:
            all_errors.append("%s: sha256 不匹配 (文件被改动过?)" % tc)
            continue
        if meta.get("emf_unit") != "uV":
            all_errors.append("%s: emf_unit 必须是 uV, 实为 %r" % (tc, meta.get("emf_unit")))
            continue
        rows = [ln.split(",") for ln in body.strip().splitlines()]
        pts = [(int(a), float(b)) for a, b in rows]
        errs = check_monotonic(pts, tc) + check_anchors(pts, tc)
        if errs:
            all_errors += errs
            print("  [FAIL] %s" % tc)
            continue
        if tc not in manifest:
            manifest[tc] = {"file": os.path.basename(path), "type": tc, "points": len(pts),
                            "temp_min": pts[0][0], "temp_max": pts[-1][0],
                            "sha256": digest, "standard": STANDARD}
        print("  [OK] %s: %d 点, %d…%d °C, sha256 %s…"
              % (tc, len(pts), pts[0][0], pts[-1][0], digest[:12]))

    manifest_path = os.path.join(TABLE_DIR, "manifest.json")
    if not args.check_only:
        with open(manifest_path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump({"schema": "TC_TABLE v1",
                       "default_type": DEFAULT_TYPE,
                       "generated": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                       "types": manifest}, fh, ensure_ascii=False, indent=2)
        print("\n[清单] %s (default_type=%s)" % (os.path.relpath(manifest_path, PROJECT_ROOT),
                                                DEFAULT_TYPE))

    print("\n" + "=" * 74)
    if all_errors:
        print("结果: 失败 %d 项" % len(all_errors))
        for e in all_errors[:20]:
            print("  - %s" % e)
        return 1
    print("结果: 全部通过 —— %d 种分度号已就绪, 默认 %s 型"
          % (len(types), DEFAULT_TYPE))
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
