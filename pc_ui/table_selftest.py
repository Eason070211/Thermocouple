#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC_TABLE v1 分度表与表引擎自检 (无界面, 不需要硬件)

覆盖:
    1. 5 张表全部能加载, sha256/单位/方向 校验通过
    2. 单调性 (电势单调不减) + 范围/点数与 manifest 一致
    3. 往返: 表格点上 T->E->T 精确; 格点之间也要准
    4. 亚格点精度: 用固件那份独立的多项式生成电势, 再用表求逆 -> 应还原温度
       (这是"表 + 插值"精度最有力的证据, 因为电势来源完全独立于表文件)
    5. pchip 与 scipy.interpolate.PchipInterpolator 逐点一致 (验证自研实现)
    6. pchip vs linear 在 1 °C 表上的差异 (说明默认值是否重要)
    7. 冷端补偿闭环 (与固件 TC_SelfTest 的第 3/4 项同一个用例)
    8. 越界返回 NaN (绝不静默外推)
    9. 单位写错必须被拒绝 (mV 冒充 µV 是最危险的错误)
   10. 篡改数据必须被 sha256 抓住
   11. N 型平坦段求逆仍返回有限值
   12. 默认分度号必须是 K

运行: python pc_ui/table_selftest.py
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import tempfile
from typing import List, Tuple

import numpy as np

try:
    from .tc_table import (DEFAULT_TYPE, MODE_LINEAR, MODE_PCHIP, TCTable, TCTableError,
                           TCTableSet, default_table_dir)
except ImportError:                   # 直接跑脚本
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from tc_table import (DEFAULT_TYPE, MODE_LINEAR, MODE_PCHIP, TCTable, TCTableError,
                          TCTableSet, default_table_dir)

_RESULTS: List[bool] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    _RESULTS.append(bool(cond))
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           ("  (%s)" % detail) if detail and not cond else ""))
    return bool(cond)


# ---------------------------------------------------------------------------
# 独立于表文件的 K 型多项式 (与固件 Core/Src/thermocouple.c 同一套 NIST 系数)
# ---------------------------------------------------------------------------
_C_LO = (0.000000000000E+00, 0.394501280250E-01, 0.236223735980E-04,
         -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10,
         -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16,
         -0.198892668780E-19, -0.163226974860E-22)
_C_HI = (-0.176004136860E-01, 0.389212049750E-01, 0.185587700320E-04,
         -0.994575928740E-07, 0.318409457190E-09, -0.560728448890E-12,
         0.560750590590E-15, -0.320207200030E-18, 0.971511471520E-22,
         -0.121047212750E-25)
_A0, _A1, _A2 = 0.118597600000E+00, -0.118343200000E-03, 0.126968600000E+03


def fw_emf_uv(temp_c: float) -> float:
    """固件 thermocouple.c 的正多项式 (µV), 用作独立参考。"""
    coefs = _C_LO if temp_c < 0.0 else _C_HI
    acc = 0.0
    for c in reversed(coefs):
        acc = acc * temp_c + c
    mv = acc
    if temp_c >= 0.0:
        d = temp_c - _A2
        mv += _A0 * math.exp(_A1 * d * d)
    return mv * 1000.0


# ---------------------------------------------------------------------------
def test_load_all() -> TCTableSet:
    print("1) 加载 5 张表 + 清单")
    tset = TCTableSet(default_table_dir())
    check("默认分度号是 K", tset.default_type == DEFAULT_TYPE, tset.default_type)
    check("5 种分度号齐备 (E/J/K/N/T)",
          tset.available_types == ["E", "J", "K", "N", "T"], str(tset.available_types))
    check("manifest.json 存在且记录了 K 为默认",
          bool(tset.manifest) and tset.manifest.get("default_type") == "K")

    for tc in tset.available_types:
        tb = tset.get(tc)
        check("%s 型: 加载+sha256+单位校验通过 (%s)" % (tc, tb.describe()),
              tb.points > 100 and tb.emf_max > tb.emf_min)
    return tset


def test_monotonic(tset: TCTableSet) -> None:
    print("2) 单调性与清单一致性")
    for tc in tset.available_types:
        tb = tset.get(tc)
        d_temp = np.diff(tb.temp)
        d_emf = np.diff(tb.emf)
        check("%s: 温度严格递增、电势单调不减" % tc,
              bool(np.all(d_temp > 0) and np.all(d_emf >= 0)))
        entry = (tset.manifest.get("types") or {}).get(tc) or {}
        if entry:
            check("%s: 点数与 manifest 一致 (%d)" % (tc, entry.get("points", -1)),
                  entry.get("points") == tb.points)
            check("%s: sha256 与 manifest 一致" % tc,
                  entry.get("sha256") == tb.meta.get("sha256"))


def local_tol_c(tb: TCTable) -> np.ndarray:
    """每个格点上的"理论最小可分辨温度" = 0.5 LSB / 局部斜率。

    ★ 为什么不能用一个固定阈值:
      NIST 表的热电势只保留到 1 µV, 而斜率 dE/dT 随温度变化很大
      (K 型在 1000 °C 约 39 µV/°C, 在 -270 °C 附近只有零点几 µV/°C)。
      所以"1 µV 折合多少 °C"必须逐点算:
          1000 °C: 1/39     ≈ 0.026 °C
          -270 °C: 1/0.3    ≈ 3 °C   (N 型甚至出现斜率为 0 的平坦段)
      这是表本身的物理下限, 任何插值算法都不可能做得更好。
    """
    slope = np.diff(tb.emf) / np.diff(tb.temp)          # µV/°C
    with np.errstate(divide="ignore", invalid="ignore"):
        half_lsb = np.where(slope > 1e-9, 0.5 * 1.0 / np.maximum(slope, 1e-9), 0.5)
    # 每个格点取左右两个区间里较宽松的那个; 平坦段给 0.5 (即"±半个格点")
    tol = np.empty(len(tb.temp))
    tol[0] = half_lsb[0]
    tol[-1] = half_lsb[-1]
    tol[1:-1] = np.maximum(half_lsb[:-1], half_lsb[1:])
    return tol


def test_roundtrip(tset: TCTableSet) -> None:
    print("3) 往返: 格点 + 格点之间 (按局部半 LSB 理论容差判定)")
    for tc in tset.available_types:
        tb = tset.get(tc)
        back = tb.emf_to_temp(tb.emf)
        err = np.abs(back - tb.temp)
        tol = local_tol_c(tb)
        worst = float(np.nanmax(err - tol))
        check("%s: 全部 %d 个格点往返误差都在理论下限内 (最大超出 %.2e °C, 该点容差 %.3f °C)"
              % (tc, tb.points, worst, float(tol[int(np.nanargmax(err))])),
              worst <= 1e-6)

    tb = tset.get("K")
    mid = tb.temp[:-1] + 0.5
    t_mid = tb.emf_to_temp(tb.temp_to_emf(mid))
    finite = np.isfinite(t_mid)
    err = np.abs(t_mid[finite] - mid[finite])
    floor = float(np.nanmax((local_tol_c(tb)[:-1] + local_tol_c(tb)[1:]) / 2.0
                            / max(1.0, 1.0)) * 0.5)
    print("       半度点最大误差 %.4f °C (1 µV 量化的理论下限约 %.4f °C)"
          % (err.max(), floor))
    check("K: 半度点往返误差接近量化下限 (< 0.05 °C)", err.max() < 0.05,
          "%.4f" % err.max())


def test_subgrid_vs_polynomial(tset: TCTableSet) -> None:
    """最有力的精度测试: 电势来自固件那份多项式, 与表文件无关。"""
    print("4) 亚格点精度 (对比固件多项式, 电势来源独立)")
    tb = tset.get("K")
    probes = np.arange(-200.0, 1372.0, 0.37)       # 故意不落在整数格点上
    emf = np.array([fw_emf_uv(float(t)) for t in probes])
    got = tb.emf_to_temp(emf, MODE_PCHIP)
    ok = np.isfinite(got)
    err = np.abs(got[ok] - probes[ok])
    check("pchip: %d 个亚格点, 最大误差 < 0.05 °C (实测 %.4f °C)"
          % (ok.sum(), err.max()), err.max() < 0.05, "%.4f" % err.max())

    got_l = tb.emf_to_temp(emf, MODE_LINEAR)
    err_l = np.abs(got_l[ok] - probes[ok])
    print("       线性插值同一批点最大误差: %.5f °C" % err_l.max())
    check("线性插值误差也 < 0.05 °C (1 °C 表足够细)",
          err_l.max() < 0.05, "%.5f" % err_l.max())


def test_vs_scipy(tset: TCTableSet) -> None:
    print("5) 自研 pchip vs scipy.PchipInterpolator")
    try:
        from scipy.interpolate import PchipInterpolator
    except ImportError:
        print("     (未安装 scipy, 跳过 —— 不影响功能, 只是少了这层交叉验证)")
        return
    tb = tset.get("K")
    # 反向: 电势 -> 温度
    ref = PchipInterpolator(tb._uniq_emf, tb._uniq_temp)
    probe = np.linspace(tb.emf_min, tb.emf_max, 4001)
    mine = tb.emf_to_temp(probe, MODE_PCHIP)
    theirs = ref(probe)
    dev = np.nanmax(np.abs(mine - theirs))
    check("电势->温度 与 scipy 最大偏差 < 1e-6 °C (实测 %.2e)" % dev, dev < 1e-6)

    # 正向: 温度 -> 电势
    ref_f = PchipInterpolator(tb.temp, tb.emf)
    probe_t = np.linspace(tb.temp_min, tb.temp_max, 3001)
    dev_f = np.nanmax(np.abs(tb.temp_to_emf(probe_t, MODE_PCHIP) - ref_f(probe_t)))
    check("温度->电势 与 scipy 最大偏差 < 1e-6 µV (实测 %.2e)" % dev_f, dev_f < 1e-6)


def test_cjc(tset: TCTableSet) -> None:
    print("6) 冷端补偿闭环 (与固件 TC_SelfTest 同用例)")
    for tc in tset.available_types:
        tb = tset.get(tc)
        hot, cold = 500.0, 25.0
        if not (tb.temp_min <= hot <= tb.temp_max):
            continue
        emf_tc = float(tb.temp_to_emf(hot)[0]) - float(tb.temp_to_emf(cold)[0])
        got = float(tb.compensate_hot_junction(emf_tc, cold)[0])
        check("%s: 热端 %.0f °C / 冷端 %.0f °C -> %.3f °C"
              % (tc, hot, cold, got), abs(got - hot) < 0.05, "%.3f" % got)

    tb = tset.get("K")
    emf500 = float(tb.temp_to_emf(500.0)[0])
    got0 = float(tb.compensate_hot_junction(emf500, 0.0)[0])
    check("K: 冷端 0 °C 时退化为直接反解 (%.3f °C)" % got0, abs(got0 - 500.0) < 0.05)


def test_bounds_and_errors(tset: TCTableSet) -> None:
    print("7) 越界 / 单位 / 篡改 防护")
    tb = tset.get("K")
    out = tb.emf_to_temp([tb.emf_min - 1.0, tb.emf_max + 1.0, 1e9, -1e9])
    check("越界电势返回 NaN (不静默外推)", bool(np.all(np.isnan(out))), str(out))
    out2 = tb.temp_to_emf([tb.temp_min - 1.0, tb.temp_max + 1.0])
    check("越界温度返回 NaN", bool(np.all(np.isnan(out2))), str(out2))

    tmp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_table_selftest")
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp, exist_ok=True)
    try:
        src = tset.get("K").path
        raw = open(src, encoding="utf-8").read()

        # (a) 单位写成 mV -> 必须拒绝
        bad = os.path.join(tmp, "tc_type_k.csv")
        open(bad, "w", encoding="utf-8").write(raw.replace("emf_unit   = uV",
                                                           "emf_unit   = mV", 1))
        try:
            TCTable.load(bad)
            check("emf_unit=mV 被拒绝", False, "竟然加载成功了")
        except TCTableError as exc:
            check("emf_unit=mV 被拒绝", "emf_unit" in str(exc), str(exc)[:60])

        # (b) 改一个数据值 -> sha256 必须抓到
        lines = raw.splitlines()
        for i, ln in enumerate(lines):
            if ln.startswith("1000,"):
                lines[i] = "1000,99999.0"           # 篡改
                break
        open(bad, "w", encoding="utf-8").write("\n".join(lines) + "\n")
        try:
            TCTable.load(bad)
            check("篡改数据被 sha256 抓住", False, "竟然加载成功了")
        except TCTableError as exc:
            check("篡改数据被 sha256 抓住", "sha256" in str(exc), str(exc)[:60])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_flat_segment(tset: TCTableSet) -> None:
    print("8) N 型平坦段 (表分辨率导致的相等电势)")
    tb = tset.get("N")
    d = np.diff(tb.emf)
    flats = int(np.count_nonzero(d == 0))
    check("N 型确实存在相邻相等电势 (%d 处)" % flats, flats >= 1)
    got = tb.emf_to_temp(tb.emf)
    check("平坦段求逆仍返回有限值且落在合理范围",
          bool(np.all(np.isfinite(got))) and
          float(np.nanmax(np.abs(got - tb.temp))) < 3.0)


def main() -> int:
    print("=" * 74)
    print("TC_TABLE v1 分度表 + 表引擎自检 (默认分度号 %s)" % DEFAULT_TYPE)
    print("=" * 74)
    try:
        tset = test_load_all()
    except TCTableError as exc:
        print("  加载失败: %s" % exc)
        print("  提示: 先运行 python tools/nist_tc_tables.py 生成 tables/")
        return 1
    test_monotonic(tset)
    test_roundtrip(tset)
    test_subgrid_vs_polynomial(tset)
    test_vs_scipy(tset)
    test_cjc(tset)
    test_bounds_and_errors(tset)
    test_flat_segment(tset)

    passed = sum(1 for r in _RESULTS if r)
    total = len(_RESULTS)
    print("-" * 74)
    print("自检结果: %d/%d 项通过 — %s" % (passed, total,
                                          "全部通过" if passed == total else "存在失败项"))
    print("=" * 74)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
