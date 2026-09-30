#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DemoSource —— 无硬件时的虚拟下位机。

它和 :class:`~pc_ui.serial_reader.SerialReader` 有完全相同的对外接口
(``start`` / ``stop`` / ``send`` / ``send_command`` / ``is_connected`` / ``out_queue``),
所以 UI 代码不需要为演示模式写任何分支。

内部行为:
    1. 接收下行帧 (用 FrameParser(HEAD_DOWN) 拆包), 响应 CMD=0x01/0x02/0x03;
    2. 按 10Hz 生成 32 路温度帧 —— **先组真实字节帧, 再喂给 FrameParser(HEAD_UP) 解析**,
       这样 CRC / 拆帧 / 解析全链路都被真实地走了一遍;
    3. 随机注入: 断线通道(NaN)、超温通道、被破坏 CRC 的坏帧 (验证丢弃逻辑);
    4. 每 1s 上报一帧状态 (CMD=0x11)。
"""

from __future__ import annotations

import math
import queue
import random
import threading
import time
from typing import Dict, Optional

try:                                  # 允许 "python pc_ui/main.py" 直接跑
    from .protocol import (CHANNEL_COUNT, CHIP_COUNT, CMD_DOWN_RUN, CMD_DOWN_SET_RATE,
                           CMD_DOWN_SINGLE, CMD_UP_STATUS, CMD_UP_TEMP, DR_TABLE,
                           HEAD_DOWN, HEAD_UP, REJECT_TABLE, STATUS_DATA_LEN,
                           FrameParser, build_frame, build_temp_frame, parse_status,
                           parse_temp)
except ImportError:                   # pragma: no cover
    from protocol import (CHANNEL_COUNT, CHIP_COUNT, CMD_DOWN_RUN, CMD_DOWN_SET_RATE,
                          CMD_DOWN_SINGLE, CMD_UP_STATUS, CMD_UP_TEMP, DR_TABLE,
                          HEAD_DOWN, HEAD_UP, REJECT_TABLE, STATUS_DATA_LEN,
                          FrameParser, build_frame, build_temp_frame, parse_status,
                          parse_temp)


class DemoSource(threading.Thread):
    """虚拟 STM32: 产生与真实下位机同格式的数据流。"""

    def __init__(self, out_queue: Optional["queue.Queue[Dict]"] = None,
                 frame_period: float = 0.1, status_period: float = 1.0,
                 corrupt_rate: float = 0.02, seed: Optional[int] = 20240521):
        super().__init__(name="DemoSource", daemon=True)
        self.out_queue: "queue.Queue[Dict]" = out_queue or queue.Queue()
        self.frame_period = frame_period
        self.status_period = status_period
        self.corrupt_rate = corrupt_rate
        self._random = random.Random(seed)

        self._stop_event = threading.Event()
        self._parser_down = FrameParser(HEAD_DOWN)
        self._parser_up = FrameParser(HEAD_UP)

        self.dr_index = 0                 # 20 SPS
        self.reject_index = 0
        self.running = True
        self.rounds = 0
        self.open_count = 0
        self._t0 = time.time()
        self._phases = [self._random.uniform(0, 2 * math.pi) for _ in range(CHANNEL_COUNT)]
        self._base = [self._random.uniform(15.0, 45.0) for _ in range(CHANNEL_COUNT)]
        #: 固定几个"故障演示"通道, 让界面上的状态列有东西可看
        self._open_channels = {7, 19}
        self._hot_channels = {3}
        self._crc_ok = 0
        self._crc_bad = 0

    # ------------------------------------------------------------------
    # 与 SerialReader 对齐的接口
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return not self._stop_event.is_set()

    def send(self, frame: bytes) -> None:
        """接收一帧下行数据并立即处理 (演示模式不需要真正的队列表)。"""
        if self._stop_event.is_set():
            return
        for cmd, payload in self._parser_down.feed(frame):
            self._handle_down_command(cmd, payload)

    def send_command(self, cmd: int, data: bytes = b"") -> None:
        self.send(build_frame(cmd, data))

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=timeout)

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _emit(self, **message) -> None:
        message.setdefault("t", time.time())
        self.out_queue.put(message)

    def _handle_down_command(self, cmd: int, payload: bytes) -> None:
        if cmd == CMD_DOWN_SET_RATE and len(payload) >= 1:
            self.dr_index = min(payload[0] & 0x07, len(DR_TABLE) - 1)
            self.reject_index = payload[1] & 0x03 if len(payload) >= 2 else 0
            # 与固件一致: 抑制只在 20SPS 下有效
            if self.dr_index != 0:
                self.reject_index = 0
            self._emit(type="status", status=self._status_dict())
        elif cmd == CMD_DOWN_RUN and len(payload) >= 1:
            self.running = bool(payload[0])
            self._emit(type="status", status=self._status_dict())
        elif cmd == CMD_DOWN_SINGLE:
            channel = payload[0] if payload else 0xFF
            self._publish_temp(single_channel=None if channel == 0xFF else channel)

    def _status_dict(self) -> Dict:
        chip_ok = 0
        chip_err = 0
        for chip in range(CHIP_COUNT):
            if chip in {c // 2 for c in self._open_channels}:
                chip_err |= 1 << chip
            else:
                chip_ok |= 1 << chip
        payload = bytearray(STATUS_DATA_LEN)
        payload[0] = 1 if self.running else 0
        payload[1] = (self.dr_index << 5) & 0xE0
        payload[2] = (self.reject_index << 4) & 0x30
        payload[3] = chip_ok & 0xFF
        payload[4] = (chip_ok >> 8) & 0xFF
        payload[5] = chip_err & 0xFF
        payload[6] = (chip_err >> 8) & 0xFF
        payload[7:11] = int(self.rounds).to_bytes(4, "little")
        payload[11:15] = int((time.time() - self._t0) * 1000).to_bytes(4, "little")
        payload[15:17] = int(self.rounds * CHANNEL_COUNT & 0xFFFF).to_bytes(2, "little")
        payload[17:19] = (0).to_bytes(2, "little")
        payload[19:21] = int(self.open_count & 0xFFFF).to_bytes(2, "little")
        payload[21] = 0
        payload[22:24] = (0).to_bytes(2, "little")
        return parse_status(bytes(payload))

    def _sample_channels(self) -> list:
        now = time.time() - self._t0
        values = []
        for ch in range(CHANNEL_COUNT):
            base = self._base[ch] + 6.0 * math.sin(2 * math.pi * 0.05 * now + self._phases[ch])
            base += self._random.gauss(0.0, 0.08)          # 测量噪声
            if ch in self._open_channels and int(now) % 12 in (5, 6, 7):
                values.append(float("nan"))                # 模拟断线
                self.open_count += 1
            elif ch in self._hot_channels:
                # 模拟超温: 稳态在 1050°C 附近 (高于默认 1000°C 门限)
                values.append(1055.0 + 25.0 * math.sin(2 * math.pi * 0.03 * now))
            else:
                values.append(base)
        return values

    def _publish_temp(self, single_channel: Optional[int] = None) -> None:
        """组真实字节帧 -> 走 FrameParser -> 投递解析结果 (与串口路径完全一致)。"""
        values = self._sample_channels()
        if single_channel is not None:
            values = [v if i == single_channel else float("nan")
                      for i, v in enumerate(values)]
        self.rounds += 1

        corrupt = self._random.random() < self.corrupt_rate
        frame = build_temp_frame(values, use_crc=not corrupt)
        if corrupt:
            self._crc_bad += 1
        else:
            self._crc_ok += 1
        for cmd, payload in self._parser_up.feed(frame):
            if cmd == CMD_UP_TEMP:
                self._emit(type="temp", t=time.time(), temps=parse_temp(payload),
                           length=len(payload))
        if self._parser_up.stats["crc_errors"]:
            self._emit(type="crc_error", count=self._parser_up.stats["crc_errors"])

    def _publish_status(self) -> None:
        payload = self._status_dict()
        frame = build_frame(CMD_UP_STATUS, payload["raw"], head=HEAD_UP)
        for cmd, payload_bytes in self._parser_up.feed(frame):
            if cmd == CMD_UP_STATUS:
                status = parse_status(payload_bytes)
                status["t"] = time.time()
                self._emit(type="status", t=time.time(), status=status)

    def run(self) -> None:
        self._emit(type="connected", port="DEMO", baudrate=460800)
        self._emit(type="info", message="演示模式: 虚拟下位机已启动 (DEMO)")
        next_frame = time.time()
        next_status = time.time()
        while not self._stop_event.is_set():
            now = time.time()
            if self.running and now >= next_frame:
                self._publish_temp()
                next_frame = now + self.frame_period
            if now >= next_status:
                self._publish_status()
                next_status = now + self.status_period
            self._stop_event.wait(0.005)
        self._emit(type="info", message="演示模式: 虚拟下位机已停止")
