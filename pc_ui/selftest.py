#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无界面自检 —— 不需要硬件、不需要打开窗口, 用来快速验证上位机各模块。

覆盖:
    1. 协议层      CRC-16/MODBUS、组帧/拆帧闭环、抗干扰、状态解析 (protocol.selftest)
    2. 环形缓冲    RingBuffer 的覆盖/顺序/resize
    3. 记录模块    DataRecorder 真写 CSV, 再用 Pandas 读回来核对行数/列名/NaN/状态文件
    4. 串口插件    SerialReader._handle_frames 的"字节流 -> 队列消息"链路
    5. 演示下位机  DemoSource 真跑一遍, 检查温度/状态帧与 CRC 丢弃计数

运行:
    python pc_ui/selftest.py
    python -m pc_ui.selftest
"""

from __future__ import annotations

import os
import queue
import shutil
import sys
import time
from typing import List

import numpy as np
import pandas as pd

try:                                  # 包内导入
    from .protocol import (CHANNEL_COUNT, CMD_DOWN_SET_RATE, STATUS_DATA_LEN,
                           build_frame, build_temp_frame, cmd_set_rate_sps,
                           crc16_modbus, selftest as protocol_selftest)
    from .recorder import STATUS_COLUMNS, TEMP_COLUMNS, DataRecorder
    from .serial_reader import SerialReader
    from .simulator import DemoSource
    from .widgets import STATE_NORMAL, STATE_OPEN, STATE_OVER, ChannelGrid, RingBuffer
except ImportError:                   # 直接跑脚本
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from protocol import (CHANNEL_COUNT, CMD_DOWN_SET_RATE, STATUS_DATA_LEN,
                          build_frame, build_temp_frame, cmd_set_rate_sps,
                          crc16_modbus, selftest as protocol_selftest)
    from recorder import STATUS_COLUMNS, TEMP_COLUMNS, DataRecorder
    from serial_reader import SerialReader
    from simulator import DemoSource
    from widgets import STATE_NORMAL, STATE_OPEN, STATE_OVER, ChannelGrid, RingBuffer

_RESULTS: List[bool] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    _RESULTS.append(bool(cond))
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           ("  (%s)" % detail) if detail and not cond else ""))
    return bool(cond)


# ---------------------------------------------------------------------------
def test_ring_buffer() -> None:
    print("2) RingBuffer 环形缓冲")
    buf = RingBuffer(capacity=3, channels=2)
    for i in range(3):
        buf.push(100.0 + i, [i, i * 10])
    t, v = buf.snapshot()
    check("未满时按插入顺序返回", list(t) == [100.0, 101.0, 102.0] and buf.count == 3)
    buf.push(103.0, [3, 30])
    t, v = buf.snapshot()
    check("满环后丢弃最旧点", list(t) == [101.0, 102.0, 103.0], str(list(t)))
    check("数值与时间同步滑动", list(v[:, 1]) == [10.0, 20.0, 30.0], str(list(v[:, 1])))
    check("last_time 正确", buf.last_time == 103.0)

    buf.resize(2)
    t, v = buf.snapshot()
    check("resize 保留最新数据", list(t) == [102.0, 103.0], str(list(t)))
    buf.resize(10)
    for i in range(12):
        buf.push(float(i), [i, i])
    t, v = buf.snapshot()
    check("放大容量后仍只保留 10 点且有序",
          len(t) == 10 and list(t[:3]) == [2.0, 3.0, 4.0], str(list(t)))


def test_recorder() -> None:
    print("3) DataRecorder 写 CSV + Pandas 回读")
    # 用工作区内的临时目录: 某些受限环境不允许写系统 temp
    tmp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_selftest_data")
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp, exist_ok=True)
    try:
        rec = DataRecorder(flush_rows=4, flush_interval=0.2)
        path = rec.start_recording(tmp, prefix="temperature")
        check("文件名符合 temperature_YYYYMMDD_HHMMSS.csv",
              os.path.basename(path).startswith("temperature_") and path.endswith(".csv"),
              os.path.basename(path))
        base = time.time()
        for row in range(10):
            temps = [20.0 + ch + row * 0.1 for ch in range(CHANNEL_COUNT)]
            temps[5] = float("nan")                     # 断线通道
            rec.submit(base + row * 0.1, temps)
        rec.submit_status(base, {"run": 1, "dr_sps": 20, "reject": "不抑制",
                                 "chip_ok": 0xFFFF, "chip_err": 0, "rounds": 7,
                                 "uptime_ms": 1234, "drdy_cnt": 1, "spi_err": 0,
                                 "open_cnt": 2, "timeout_cnt": 0, "flags": 0,
                                 "flag_text": "正常"})
        info = rec.stop_recording()

        check("记录行数统计正确", info["rows"] == 10, str(info["rows"]))
        check("未发生队列丢弃", info["dropped"] == 0)
        check("写盘无错误", info["error"] is None, str(info["error"]))
        if not check("CSV 文件已生成", bool(info["temp_path"] and
                                           os.path.exists(info["temp_path"]))):
            return
        frame = pd.read_csv(info["temp_path"])
        check("CSV 行数 = 10", len(frame) == 10, str(len(frame)))
        check("CSV 列名符合约定", list(frame.columns) == TEMP_COLUMNS)
        check("NaN 通道被写成空/NaN", bool(np.isnan(frame["ch05"]).all()))
        check("数值列被正常保留", abs(frame["ch00"].iloc[0] - 20.0) < 1e-6)
        check("elapsed_s 单调递增", bool((frame["elapsed_s"].diff().dropna() >= 0).all()))
        check("状态文件单独写出", bool(info["status_path"] and
                                      os.path.exists(info["status_path"])))
        status_frame = pd.read_csv(info["status_path"])
        check("状态 CSV 列名符合约定", list(status_frame.columns) == STATUS_COLUMNS)
        check("状态 CSV 有 1 行", len(status_frame) == 1, str(len(status_frame)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_serial_reader_plumbing() -> None:
    print("4) SerialReader 字节流 -> 队列消息 (不开真串口)")
    out: "queue.Queue[dict]" = queue.Queue()
    reader = SerialReader("COM_SELFTEST", 115200, out)   # 不 start(), 只测处理函数

    temps = [18.0 + i for i in range(CHANNEL_COUNT)]
    temps[31] = float("nan")
    good = build_temp_frame(temps)
    bad = bytearray(good)
    bad[20] ^= 0xFF                                     # 破坏一个字节
    # 一段"垃圾 + 坏帧 + 好帧"的真实字节流
    reader._handle_frames(b"\x13\x37" + bytes(bad) + good)

    kinds = []
    temps_back = None
    while True:
        try:
            msg = out.get_nowait()
        except queue.Empty:
            break
        kinds.append(msg["type"])
        if msg["type"] == "temp":
            temps_back = msg["temps"]

    check("坏帧被 CRC 丢弃, 只有 1 帧温度进入队列",
          kinds.count("temp") == 1, str(kinds))
    check("CRC 错误计数被上报", any(k == "crc_error" for k in kinds), str(kinds))
    check("温度数值解析正确",
          temps_back is not None and abs(temps_back[0] - 18.0) < 1e-6
          and np.isnan(temps_back[31]))

    payload = bytearray(STATUS_DATA_LEN)
    payload[0] = 1
    reader._handle_frames(build_frame(0x11, bytes(payload), head=0x55))
    status_msg = None
    while True:
        try:
            msg = out.get_nowait()
        except queue.Empty:
            break
        if msg["type"] == "status":
            status_msg = msg
    check("状态帧被解析进队列", status_msg is not None and status_msg["status"]["run"] == 1)


def test_demo_source() -> None:
    print("5) DemoSource 虚拟下位机 (端到端)")
    out: "queue.Queue[dict]" = queue.Queue()
    src = DemoSource(out, frame_period=0.02, status_period=0.1, corrupt_rate=0.15)
    src.start()
    src.send_command(0x01, bytes((2, 0)))               # 设置 90 SPS
    src.send_command(0x02, bytes((1,)))                 # 启动采集

    deadline = time.time() + 5.0
    temp_msgs, status_msgs, crc_msgs = [], [], []
    # 坏帧是随机注入的, 所以一直收, 直到三样都出现或超时
    while time.time() < deadline and not (len(temp_msgs) >= 5 and status_msgs and crc_msgs):
        try:
            msg = out.get_nowait()
        except queue.Empty:
            time.sleep(0.01)
            continue
        temp_msgs.append(msg) if msg["type"] == "temp" else None
        status_msgs.append(msg) if msg["type"] == "status" else None
        crc_msgs.append(msg) if msg["type"] == "crc_error" else None
    src.stop()

    check("收到 >=5 帧温度", len(temp_msgs) >= 5, str(len(temp_msgs)))
    check("温度帧长 32 路", all(len(m["temps"]) == CHANNEL_COUNT for m in temp_msgs))
    check("收到状态帧", bool(status_msgs))
    if status_msgs:
        check("采样率设置生效 (90 SPS)",
              status_msgs[-1]["status"]["dr_sps"] == 90,
              str(status_msgs[-1]["status"].get("dr_sps")))
    check("坏帧被 CRC 拦截并计数", bool(crc_msgs))


def test_state_classification() -> None:
    print("6) 通道状态判定 (正常/断线/超温)")
    # 只测纯逻辑, 不创建 Tk 控件
    class _Probe:
        classify = ChannelGrid.classify
        def __init__(self, over_temp):
            self.over_temp = over_temp
    probe = _Probe(1000.0)
    check("NaN -> 断线", probe.classify(float("nan")) == STATE_OPEN)
    check("25°C -> 正常", probe.classify(25.0) == STATE_NORMAL)
    check("1100°C -> 超温", probe.classify(1100.0) == STATE_OVER)
    check("恰好等于门限 -> 正常", probe.classify(1000.0) == STATE_NORMAL)


# ---------------------------------------------------------------------------
def main() -> int:
    print("=" * 74)
    print("32 路热电偶上位机自检 (无界面, 不需要硬件)")
    print("=" * 74)

    print("1) 协议层 (protocol.py)")
    ok = protocol_selftest(verbose=True)
    _RESULTS.append(ok)

    test_ring_buffer()
    test_recorder()
    test_serial_reader_plumbing()
    test_demo_source()
    test_state_classification()

    passed = sum(1 for r in _RESULTS if r)
    total = len(_RESULTS)
    print("-" * 74)
    print("自检结果: %d/%d 项通过 — %s" % (passed, total, "全部通过" if passed == total else "存在失败项"))
    print("=" * 74)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
