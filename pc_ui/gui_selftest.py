#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
界面集成自检 —— 真的把 TemperatureApp 拉起来 (会短暂弹窗), 用代码模拟点击,
验证"演示下位机 -> 队列 -> 曲线/数值 -> CSV 落盘 -> 暂停/勾选/单次读取"整条链路。

和 selftest.py 的区别:
    selftest.py    不需要显示器, 只测协议/缓冲/记录/线程
    gui_selftest.py 需要图形环境 (Windows/macOS/Linux 桌面), 测的是 UI 的接线

运行:
    python pc_ui/gui_selftest.py
退出码 0 = 全部通过。
"""

from __future__ import annotations

import os
import shutil
import sys
import time

import numpy as np

try:                                  # 包内导入
    from .app import TemperatureApp
    from .recorder import CHANNEL_COUNT
except ImportError:                   # 直接跑脚本
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from app import TemperatureApp
    from recorder import CHANNEL_COUNT

import pandas as pd

TMP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_gui_selftest_data")
#: 设 TC_KEEP_TMP=1 可保留自检产生的 CSV, 便于排查问题
KEEP_TMP = bool(os.environ.get("TC_KEEP_TMP"))
_RESULTS = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    _RESULTS.append(bool(cond))
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           ("  (%s)" % detail) if detail and not cond else ""))
    return bool(cond)


def pump(app: TemperatureApp, seconds: float, step: float = 0.1) -> None:
    """代替 mainloop: 手动跑 Tk 事件循环, 让 root.after 回调有机会执行。"""
    deadline = time.time() + seconds
    while time.time() < deadline:
        app.root.update()
        time.sleep(step)


def main() -> int:
    print("=" * 74)
    print("上位机界面集成自检 (会短暂弹出窗口, 使用演示下位机)")
    print("=" * 74)

    shutil.rmtree(TMP_DIR, ignore_errors=True)
    os.makedirs(TMP_DIR, exist_ok=True)
    if KEEP_TMP:
        print("  (TC_KEEP_TMP=1: 自检产生的 CSV 将保留在 %s)" % TMP_DIR)

    app = TemperatureApp(demo=True, auto_connect=False, retention=300,
                         record_dir=TMP_DIR, over_temp=1000.0)
    app.root.update()
    check("窗口创建成功 (含 32 条曲线)", len(app.lines) == CHANNEL_COUNT)
    check("32 个通道勾选框", len(app.selector.vars) == CHANNEL_COUNT)

    # ---- 连接演示下位机 ----
    app.connect()
    app.reader.corrupt_rate = 0.4          # 提高坏帧注入比例, 让 CRC 丢弃必然发生
    pump(app, 2.5)
    check("已连接虚拟下位机", app.reader is not None)
    check("收到温度帧", app.temp_frames > 5, "帧数=%d" % app.temp_frames)
    check("环形缓冲有数据", app.ring.count > 5, "点数=%d" % app.ring.count)
    check("数值面板拿到最新值", int(np.count_nonzero(~np.isnan(app.latest))) > 0)
    check("曲线已生成 X 轴数据", app.lines[0].get_xdata().size > 0)
    check("收到状态帧", app.status is not None and app.status.get("dr_sps") is not None)
    check("CRC 丢弃计数在刷新", app.crc_errors >= 1, str(app.crc_errors))
    check("帧率统计 > 0", app.stat_vars["fps"].get() not in ("-", "0.0 Hz"))
    print("     ylim=%s" % (app.ax.get_ylim(),))

    # ---- 采样率切换 (CMD=0x01) ----
    app.rate_var.set("600 SPS")
    app.reject_var.set("不抑制")
    app.apply_rate()
    pump(app, 1.6)
    check("采样率切换后状态帧回读 600 SPS",
          app.status and app.status.get("dr_sps") == 600,
          str(app.status and app.status.get("dr_sps")))

    app.rate_var.set("20 SPS")
    app.reject_var.set("同时抑制50+60Hz")
    app.apply_rate()
    pump(app, 1.6)
    check("抑制设置回读正确",
          app.status and app.status.get("dr_sps") == 20
          and app.status.get("reject") == "同时抑制50+60Hz",
          str(app.status and (app.status.get("dr_sps"), app.status.get("reject"))))

    # ---- 停止 / 启动 / 单次读取 ----
    app.acquire(False)
    pump(app, 1.2)
    check("停止采集后下位机 run=0", app.status and app.status.get("run") == 0)
    before = app.temp_frames
    pump(app, 0.6)
    check("停止后不再产生温度帧", app.temp_frames == before,
          "%d -> %d" % (before, app.temp_frames))
    points_before = app.ring.count
    app.single_read()
    pump(app, 1.2)
    valid_now = int(np.count_nonzero(~np.isnan(app.latest)))
    check("单次读取返回结果 (只更新数值面板)",
          valid_now >= CHANNEL_COUNT - 2 and app.ring.count == points_before,
          "valid=%d, ring %d -> %d" % (valid_now, points_before, app.ring.count))
    app.acquire(True)
    pump(app, 1.5)
    check("重新启动采集后 run=1", app.status and app.status.get("run") == 1)

    # ---- 绘图交互 ----
    app.paused = False
    app.toggle_pause()
    check("暂停按钮状态切换", app.paused and app.pause_btn.cget("text") == "继续")
    app.on_visibility_change(np.zeros(CHANNEL_COUNT, dtype=bool))
    app._update_plot()
    check("全部隐藏后曲线不可见", not app.lines[0].get_visible())
    app.selector._set_all(True)
    app._update_plot()
    check("全选后曲线可见", app.lines[0].get_visible())
    app.toggle_pause()
    check("继续按钮状态切换", (not app.paused) and app.pause_btn.cget("text") == "暂停")
    app.retention_var.set(120)
    app.on_retention_change()
    check("保留点数生效", app.ring.capacity == 120, str(app.ring.capacity))
    app.over_temp_var.set(500.0)
    app.on_over_temp_change()
    check("超温门限生效", app.over_temp == 500.0 and app.grid_view.over_temp == 500.0)
    app.over_temp_var.set(1000.0)
    app.on_over_temp_change()

    # ---- CSV 记录 ----
    app.start_recording()
    check("记录已启动", app.recorder.active and app.record_btn.cget("text") == "停止记录")
    pump(app, 2.5)
    check("记录过程中有新行写入", app.recorder.rows_written > 5,
          str(app.recorder.rows_written))
    rows_written = app.recorder.rows_written
    app.stop_recording()
    rows_written = app.recorder.rows_written       # 停止时会把尾巴刷盘, 行数以此为准
    check("记录已停止", not app.recorder.active)
    csv_path = app.recorder.temp_path
    check("CSV 文件存在", bool(csv_path and os.path.exists(csv_path)), str(csv_path))
    if csv_path and os.path.exists(csv_path):
        frame = pd.read_csv(csv_path)
        check("CSV 行数 = 记录行数", len(frame) == rows_written,
              "%d vs %d" % (len(frame), rows_written))
        check("CSV 含 32 路通道列",
              all(("ch%02d" % i) in frame.columns for i in range(CHANNEL_COUNT)))
        check("CSV 至少有 1 路有效温度",
              int(frame[[c for c in frame.columns if c.startswith("ch")]].notna()
                  .to_numpy().sum()) > 0)
    status_path = app.recorder.status_path
    check("状态 CSV 同时生成", bool(status_path and os.path.exists(status_path)))

    # ---- 断开 ----
    app.disconnect()
    pump(app, 0.4)
    check("断开后内部分支清空", app.reader is None)

    app.on_close()
    if not KEEP_TMP:
        shutil.rmtree(TMP_DIR, ignore_errors=True)

    passed = sum(1 for r in _RESULTS if r)
    total = len(_RESULTS)
    print("-" * 74)
    print("界面自检结果: %d/%d 项通过 — %s"
          % (passed, total, "全部通过" if passed == total else "存在失败项"))
    print("=" * 74)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
