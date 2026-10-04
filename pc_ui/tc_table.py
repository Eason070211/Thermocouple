#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC_TABLE v1 分度表运行时引擎 (纯 numpy, 不依赖 scipy)

职责
----
    1. 加载 tables/tc_type_<x>.csv (TC_TABLE v1)
    2. 校验: sha256 / 单位 / 方向 / 单调性
    3. 双向换算:
         temp_to_emf(t)   温度 °C  -> 热电势 µV   (分度表方向)
         emf_to_temp(uv)  热电势 µV -> 温度 °C   (测量方向)
    4. 冷端补偿: hot = emf_to_temp(emf_tc + E(cj))
       —— 与固件 Core/Src/thermocouple.c 的 TC_Compute() 完全同一个算法,
          只是那边用多项式, 这边用表 + 插值。

插值方式
--------
    mode="pchip"  (默认) 单调三次 Hermite (Fritsch–Carlson) —— 不会过冲,
                 对**粗表**(厂家给的 10 °C 甚至 100 °C 步长表) 明显更准
    mode="linear"         线性插值 —— 最快, 精度有保证时用

    ★ 实测说明: NIST 的 1 °C 步长表上, 线性插值误差 < 0.001 °C, 两种方式
      没有实际差别; 默认给 pchip 是为了兼容以后导入的粗粒度厂家表。

为什么不用 scipy
----------------
    scipy 是几十 MB 的重依赖, 而这个文件只有 20 KB。pchip 的系数算法
    (Fritsch–Carlson) 很短且稳定, 自己实现反而让部署更简单。
    自检脚本 pc_ui/table_selftest.py 里用 scipy 的 PchipInterpolator
    做了逐点交叉验证, 确认两者数值一致。

默认分度号: K (见 DEFAULT_TYPE)
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: 默认分度号 —— 整个工程都以 K 型为准
DEFAULT_TYPE = "K"

#: 支持的插值方式
MODE_PCHIP = "pchip"
MODE_LINEAR = "linear"
VALID_MODES = (MODE_PCHIP, MODE_LINEAR)

_TABLE_FILE_RE = re.compile(r"^tc_type_([a-z])\.csv$", re.I)


# ---------------------------------------------------------------------------
# 单调三次 Hermite 插值 (Fritsch–Carlson), 与 scipy PchipInterpolator 等价
# ---------------------------------------------------------------------------
def _pchip_slopes(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """求单调三次 Hermite 各节点斜率 d[]。要求 x 严格递增。"""
    n = len(x)
    if n == 2:
        return np.array([(y[1] - y[0]) / (x[1] - x[0])] * 2, dtype=float)

    h = np.diff(x)
    delta = np.diff(y) / h
    d = np.zeros(n, dtype=float)

    # 内部节点: delta 同号时取加权调和平均, 异号(极值点)取 0 -> 不过冲
    for i in range(1, n - 1):
        if delta[i - 1] * delta[i] > 0.0:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            d[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i])
        else:
            d[i] = 0.0

    # 端点: scipy 用的三点公式 + 单调性钳位
    def _endpoint(h0: float, h1: float, d0: float, d1: float) -> float:
        val = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if val * d0 <= 0.0:
            return 0.0
        if d0 * d1 < 0.0 and abs(val) > 3.0 * abs(d0):
            return 3.0 * d0
        return val

    d[0] = _endpoint(h[0], h[1], delta[0], delta[1])
    d[-1] = _endpoint(h[-1], h[-2], delta[-1], delta[-2])
    return d


class _Interp:
    """一维单调插值器: 预计算斜率, 求值时用 searchsorted + 局部 Hermite。"""

    __slots__ = ("x", "y", "d", "mode", "_xmin", "_xmax")

    def __init__(self, x: Sequence[float], y: Sequence[float], mode: str = MODE_PCHIP):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.mode = mode
        if len(self.x) < 2:
            raise ValueError("插值至少需要 2 个点")
        self._xmin = float(self.x[0])
        self._xmax = float(self.x[-1])
        self.d = _pchip_slopes(self.x, self.y) if mode == MODE_PCHIP else np.zeros_like(self.x)

    def __call__(self, xq) -> np.ndarray:
        """求值; 超出 [xmin, xmax] 的点返回 NaN (不静默外推)。"""
        q = np.atleast_1d(np.asarray(xq, dtype=float))
        out = np.full(q.shape, np.nan, dtype=float)

        inside = np.isfinite(q) & (q >= self._xmin) & (q <= self._xmax)
        if not np.any(inside):
            return out

        qi = q[inside]
        idx = np.searchsorted(self.x, qi, side="right") - 1
        idx = np.clip(idx, 0, len(self.x) - 2)

        x0 = self.x[idx]
        x1 = self.x[idx + 1]
        y0 = self.y[idx]
        y1 = self.y[idx + 1]
        h = x1 - x0

        if self.mode == MODE_LINEAR:
            out[inside] = y0 + (y1 - y0) * (qi - x0) / h
        else:
            t = (qi - x0) / h
            d0 = self.d[idx]
            d1 = self.d[idx + 1]
            # 三次 Hermite 基函数
            h00 = (1.0 + 2.0 * t) * (1.0 - t) ** 2
            h10 = t * (1.0 - t) ** 2
            h01 = t * t * (3.0 - 2.0 * t)
            h11 = t * t * (t - 1.0)
            out[inside] = h00 * y0 + h10 * h * d0 + h01 * y1 + h11 * h * d1

        return out

    @property
    def xmin(self) -> float:
        return self._xmin

    @property
    def xmax(self) -> float:
        return self._xmax


# ---------------------------------------------------------------------------
# 单张分度表
# ---------------------------------------------------------------------------
class TCTableError(Exception):
    """表文件格式/校验错误。"""


class TCTable:
    """一张 TC_TABLE v1 分度表 (含双向插值器)。"""

    def __init__(self, meta: Dict[str, str], temp_c: np.ndarray, emf_uv: np.ndarray,
                 path: str = "", verify_hash: bool = True, body_sha256: str = ""):
        self.meta = dict(meta)
        self.path = path
        self.tc_type = str(meta.get("type", "?")).upper()
        self.direction = str(meta.get("direction", "T2E")).upper()

        if str(meta.get("emf_unit", "")).strip().lower() not in ("uv", "µv", "microvolt"):
            raise TCTableError("%s: emf_unit 必须是 uV (实为 %r) —— 单位错了温度会差几个"
                               "数量级, 这是本格式强制字段" % (path, meta.get("emf_unit")))
        if str(meta.get("temp_unit", "")).strip().upper() not in ("C", "°C", "DEGC"):
            raise TCTableError("%s: temp_unit 必须是 C (实为 %r)" % (path, meta.get("temp_unit")))

        if len(temp_c) < 2 or len(temp_c) != len(emf_uv):
            raise TCTableError("%s: 数据点非法" % path)
        if not np.all(np.diff(temp_c) > 0):
            raise TCTableError("%s: 温度列必须严格递增" % path)

        # sha256 校验: 防止有人手改表里的数字
        if verify_hash and body_sha256:
            want = str(meta.get("sha256", "")).strip()
            got = hashlib.sha256(body_sha256.encode("utf-8")).hexdigest()
            if want and want != got:
                raise TCTableError("%s: sha256 不匹配 (期望 %s…, 实得 %s…) —— "
                                   "表文件被改动过" % (path, want[:12], got[:12]))

        if self.direction == "E2T":
            emf_uv, temp_c = temp_c, emf_uv          # 列顺序与 T2E 相反
            if not np.all(np.diff(emf_uv) > 0):
                raise TCTableError("%s: E2T 表的电势列必须严格递增" % path)

        self.temp = temp_c.astype(float)
        self.emf = emf_uv.astype(float)
        if not np.all(np.diff(self.emf) >= 0):
            raise TCTableError("%s: 电势列必须单调不减" % path)

        self.points = len(self.temp)
        self.temp_min = float(self.temp[0])
        self.temp_max = float(self.temp[-1])
        self.emf_min = float(self.emf[0])
        self.emf_max = float(self.emf[-1])

        # ---- 正向 (温度 -> 电势): 直接插值 ----
        self._fwd: Dict[str, _Interp] = {}

        # ---- 反向 (电势 -> 温度) ----
        # 电势可能有相等值 (NIST 表 1 µV 分辨率 + N 型在 -270 °C 附近热电率极小),
        # 相等的段落在求逆时无法区分, 取该段温度的均值 —— 误差远小于表分辨率本身。
        uniq_emf: List[float] = []
        uniq_temp: List[float] = []
        i = 0
        while i < self.points:
            j = i
            while j + 1 < self.points and self.emf[j + 1] == self.emf[i]:
                j += 1
            uniq_emf.append(float(self.emf[i]))
            uniq_temp.append(float(self.temp[i:j + 1].mean()))
            i = j + 1
        self._inv_points = len(uniq_emf)
        self._uniq_emf = np.asarray(uniq_emf, dtype=float)
        self._uniq_temp = np.asarray(uniq_temp, dtype=float)
        self._inv: Dict[str, _Interp] = {}

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path: str, verify_hash: bool = True) -> "TCTable":
        """读取一个 TC_TABLE v1 文件。"""
        with open(path, encoding="utf-8") as fh:
            lines = fh.read().splitlines()

        meta: Dict[str, str] = {}
        body_start = None
        for idx, line in enumerate(lines):
            if line.startswith("#"):
                item = line[1:].strip()
                if "=" in item and not item.startswith("TC_TABLE"):
                    key, _, val = item.partition("=")
                    meta[key.strip()] = val.split("(")[0].strip()
            elif line.strip():
                body_start = idx + 1
                break
        if body_start is None:
            raise TCTableError("%s: 找不到表头行/数据体" % path)

        body_lines = [ln for ln in lines[body_start:] if ln.strip()]
        body_text = "\n".join(body_lines) + "\n"

        temps: List[float] = []
        emfs: List[float] = []
        for lineno, ln in enumerate(body_lines, body_start + 1):
            parts = ln.split(",")
            if len(parts) != 2:
                raise TCTableError("%s:%d 每行必须是 2 列, 实为 %d 列"
                                   % (path, lineno, len(parts)))
            try:
                temps.append(float(parts[0]))
                emfs.append(float(parts[1]))
            except ValueError:
                raise TCTableError("%s:%d 数值解析失败: %r" % (path, lineno, ln)) from None

        return cls(meta, np.asarray(temps), np.asarray(emfs), path=path,
                   verify_hash=verify_hash, body_sha256=body_text)

    # ------------------------------------------------------------------
    def _forward(self, mode: str) -> _Interp:
        if mode not in self._fwd:
            self._fwd[mode] = _Interp(self.temp, self.emf, mode)
        return self._fwd[mode]

    def _inverse(self, mode: str) -> _Interp:
        if mode not in self._inv:
            self._inv[mode] = _Interp(self._uniq_emf, self._uniq_temp, mode)
        return self._inv[mode]

    # ------------------------------------------------------------------
    def temp_to_emf(self, temp_c, mode: str = MODE_PCHIP) -> np.ndarray:
        """温度 °C -> 热电势 µV (参考端 0 °C)。超出表范围返回 NaN。"""
        if mode not in VALID_MODES:
            raise ValueError("未知插值方式: %r" % mode)
        return self._forward(mode)(temp_c)

    def emf_to_temp(self, emf_uv, mode: str = MODE_PCHIP) -> np.ndarray:
        """热电势 µV -> 温度 °C。超出表范围返回 NaN。"""
        if mode not in VALID_MODES:
            raise ValueError("未知插值方式: %r" % mode)
        return self._inverse(mode)(emf_uv)

    def compensate_hot_junction(self, emf_tc_uv, cj_c, mode: str = MODE_PCHIP) -> np.ndarray:
        """冷端补偿: 由"实测热电势 + 冷端温度"算出热端温度 °C。

        与固件 TC_Compute() 同一个公式:
            V_total = V_TC + E(T_cj)      (把冷端等效电势补回去)
            T_hot   = E^-1(V_total)
        """
        emf_cj = self.temp_to_emf(cj_c, mode)
        return self.emf_to_temp(np.asarray(emf_tc_uv, dtype=float) + emf_cj, mode)

    # ------------------------------------------------------------------
    def describe(self) -> str:
        """一行摘要 (给 UI 状态栏用)。"""
        return ("%s 型 %d 点 %.0f…%.0f °C  (%.0f…%.0f µV)"
                % (self.tc_type, self.points, self.temp_min, self.temp_max,
                   self.emf_min, self.emf_max))

    def __repr__(self) -> str:
        return "<TCTable %s>" % self.describe()


# ---------------------------------------------------------------------------
# 表集合: 扫描目录 / 默认 K / 加载缓存
# ---------------------------------------------------------------------------
class TCTableSet:
    """管理 tables/ 目录下的所有分度表, 默认使用 K 型。"""

    def __init__(self, directory: str, default_type: str = DEFAULT_TYPE,
                 verify_hash: bool = True):
        self.directory = directory
        self.default_type = default_type.upper()
        self.verify_hash = verify_hash
        self._cache: Dict[str, TCTable] = {}
        self._files: Dict[str, str] = {}
        self._errors: Dict[str, str] = {}
        self.manifest: Dict = {}
        self._scan()

    def _scan(self) -> None:
        self._files.clear()
        if not os.path.isdir(self.directory):
            return
        for name in sorted(os.listdir(self.directory)):
            m = _TABLE_FILE_RE.match(name)
            if m:
                self._files[m.group(1).upper()] = os.path.join(self.directory, name)
        mpath = os.path.join(self.directory, "manifest.json")
        if os.path.exists(mpath):
            try:
                with open(mpath, encoding="utf-8") as fh:
                    self.manifest = json.load(fh)
                self.default_type = str(self.manifest.get("default_type",
                                                          self.default_type)).upper()
            except (OSError, ValueError):
                self.manifest = {}

    # ------------------------------------------------------------------
    @property
    def available_types(self) -> List[str]:
        return sorted(self._files.keys())

    @property
    def errors(self) -> Dict[str, str]:
        return dict(self._errors)

    def get(self, tc_type: Optional[str] = None) -> TCTable:
        """取一张表 (默认 K 型)。加载失败会抛 TCTableError。"""
        tc = (tc_type or self.default_type or DEFAULT_TYPE).upper()
        if tc in self._cache:
            return self._cache[tc]
        path = self._files.get(tc)
        if path is None:
            raise TCTableError("没有 %s 型分度表 (目录 %s, 现有: %s)"
                               % (tc, self.directory, ", ".join(self.available_types) or "无"))
        try:
            table = TCTable.load(path, verify_hash=self.verify_hash)
        except TCTableError as exc:
            self._errors[tc] = str(exc)
            raise
        self._cache[tc] = table
        return table

    def try_get(self, tc_type: Optional[str] = None) -> Optional[TCTable]:
        """取表, 失败返回 None (不抛异常, 给 UI 用)。"""
        try:
            return self.get(tc_type)
        except (TCTableError, OSError) as exc:
            self._errors[(tc_type or self.default_type).upper()] = str(exc)
            return None


def default_table_dir() -> str:
    """工程内 tables/ 目录 (pc_ui/tc_table.py -> ../tables)。"""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tables")


if __name__ == "__main__":                    # 手工试一下: python -m pc_ui.tc_table [K]
    import sys
    tset = TCTableSet(default_table_dir())
    want = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TYPE
    print("可用分度号:", ", ".join(tset.available_types), " 默认:", tset.default_type)
    tb = tset.get(want)
    print("已加载:", tb.describe())
    print("路径:", os.path.relpath(tb.path))
    for probe in (0.0, 25.0, 100.0, 1000.0):
        uv = float(tb.temp_to_emf(probe)[0])
        back = float(tb.emf_to_temp(uv)[0])
        print("  %8.1f °C -> %10.2f µV -> %8.3f °C" % (probe, uv, back))
    print("冷端补偿: 热端 1000 °C / 冷端 25 °C ->",
          "%.3f °C" % float(tb.compensate_hot_junction(
              float(tb.temp_to_emf(1000.0)[0]) - float(tb.temp_to_emf(25.0)[0]), 25.0)[0]))
