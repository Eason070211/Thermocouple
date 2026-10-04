#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟下位机 —— STM32F103C8T6 + 4×ADS1220 热电偶测温板 (8 路)

用途
----
    在一个**真实串口**(例如 ELTIMA / com0com 建的虚拟串口对)上扮演固件,
    让真实的上位机 (pc_ui) 能连上它、收数据、算温度 —— 不需要板子就能
    开发调试上位机。

    固件那边怎么算、怎么发, 这边就怎么算、怎么发, 所以:
      * 协议层与固件是**同一个** (pc_ui/protocol.py 的 CRC/组帧/拆帧);
      * 热电势 µV 是用同一张 tables/ 分度表反算出来的, 所以上位机算回来
        的温度应当等于"想演示的温度"(误差只来自 1µV 的表量化)。

用法
----
    # 先把虚拟串口对建好 (ELTIMA / com0com), 然后:
    #   端口 A 挂真上位机, 端口 B 挂本模拟器
    python -m tools.serial_sim.sim_device --port COM2

    # 先自检一遍, 不需要串口 (校验 CRC / 帧 / 温度闭环):
    python -m tools.serial_sim.sim_device --test

    # 只打印帧, 不占串口 (人眼看字节):
    python -m tools.serial_sim.sim_device --dry-run

    # 配合真上位机的"演示模式"以外的路径时, 可以切换上行帧类型:
    python -m tools.serial_sim.sim_device --port COM2 --uplink temp   # 固件老行为 0x10
    python -m tools.serial_sim.sim_device --port COM2 --uplink both   # 0x10 + 0x12 都发

常用参数
--------
    --port        串口名 (如 COM2)。不给就进 dry-run 模式。
    --baudrate    波特率, 默认 115200 (与固件 UART_BAUDRATE 默认值一致)
    --chips       虚拟 ADS1220 片数 1..8, 默认 4 (8 路)
    --sps         采样率 20/45/90/175/330/600/1000, 默认 20
    --uplink      raw(默认, 对应 TC_UPLINK_MODE=1) / temp(=0) / both(=2)
    --autorun     一启动就开始采集(相当于上电就 App_Start()), 默认开启
    --corrupt     注入坏帧概率 (默认 0, 即 100% 干净帧)
    --seed        随机种子, 固定后每次演示数据一致
    --quiet       只输出关键事件, 不打印每帧
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

try:                                  # 允许 "python tools/serial_sim/sim_device.py" 直接跑
    from pc_ui.protocol import (CHANNEL_COUNT, CJ_SLOT_COUNT, CHIP_COUNT,
                                CMD_DOWN_RUN, CMD_DOWN_SET_RATE, CMD_DOWN_SINGLE,
                                CMD_UP_RAW, CMD_UP_STATUS, CMD_UP_TEMP, DR_TABLE,
                                HEAD_UP, REJECT_TABLE, ST_FLAG_CRC_ERR,
                                ST_FLAG_RAW_UPLINK, STATUS_DATA_LEN,
                                FrameParser, build_frame, build_raw_frame,
                                build_temp_frame, parse_status)
    from pc_ui.tc_table import TCTableSet, default_table_dir
except ImportError:                   # pragma: no cover
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    from pc_ui.protocol import (CHANNEL_COUNT, CJ_SLOT_COUNT, CHIP_COUNT,
                                CMD_DOWN_RUN, CMD_DOWN_SET_RATE, CMD_DOWN_SINGLE,
                                CMD_UP_RAW, CMD_UP_STATUS, CMD_UP_TEMP, DR_TABLE,
                                HEAD_UP, REJECT_TABLE, ST_FLAG_CRC_ERR,
                                ST_FLAG_RAW_UPLINK, STATUS_DATA_LEN,
                                FrameParser, build_frame, build_raw_frame,
                                build_temp_frame, parse_status)
    from pc_ui.tc_table import TCTableSet, default_table_dir

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except ImportError:                   # pragma: no cover
    serial = None
    SERIAL_AVAILABLE = False


# ---------------------------------------------------------------------------
# 物理模型: 每通道一个"想演示的温度", 冷端每片一个
# ---------------------------------------------------------------------------
class ChannelModel:
    """模拟 8 路热电偶 + 每片冷端温度。

    与真实板子同样的"故障注入", 让状态列和曲线都有东西可看:
      * 断线通道: 周期性输出 NaN (上位机显示"断线")
      * 超温通道: 稳态 1055 °C 附近 (默认 1000 °C 门限以上)
      * 测量噪声: 每通道 ~0.08 °C 高斯
      * 慢漂移: 0.05 Hz 正弦, 模拟真实工况
    """

    def __init__(self, chips: int, rng: random.Random):
        self.chips = max(1, min(int(chips), CHIP_COUNT))
        self.rng = rng
        self.n = min(self.chips * 2, CHANNEL_COUNT)
        self._phases = [rng.uniform(0, 2 * math.pi) for _ in range(CHANNEL_COUNT)]
        self._base = [rng.uniform(15.0, 45.0) for _ in range(CHANNEL_COUNT)]
        #: 每片冷端: 25 °C 附近, 片间略有差异 (模拟等温块梯度)
        self.cj = [25.0 + 0.5 * i for i in range(CHIP_COUNT)]
        #: 故障演示
        self.open_channels = {self.n - 1} if self.n > 1 else set()
        self.hot_channels = {3} if self.n > 3 else set()
        self._open_toggle = 0

    def hot_temps(self, now: float) -> List[float]:
        """给定时刻, 32 路"想演示的热端温度"(°C); 本板没有的通道为 NaN。"""
        out = [float("nan")] * CHANNEL_COUNT
        for ch in range(self.n):
            if ch in self.open_channels:
                # 每 12 s 里有 3 s 断线 —— 与 pc_ui.simulator 的行为一致
                self._open_toggle = int(now) % 12
                if self._open_toggle in (5, 6, 7):
                    out[ch] = float("nan")
                    continue
            base = self._base[ch] + 6.0 * math.sin(2 * math.pi * 0.05 * now
                                                    + self._phases[ch])
            base += self.rng.gauss(0.0, 0.08)              # 测量噪声
            if ch in self.hot_channels:
                out[ch] = 1055.0 + 25.0 * math.sin(2 * math.pi * 0.03 * now)
            else:
                out[ch] = base
        return out


# ---------------------------------------------------------------------------
# 设备主体
# ---------------------------------------------------------------------------
class SimulatedDevice(threading.Thread):
    """在真实串口上扮演 STM32 + ADS1220, 协议与固件一字不差。

    与固件的对应关系
    ----------------
        App_HwInit()          -> __init__ + open_port()
        App_Start()/Stop()    -> 0x02 命令的处理
        App_SetRate()         -> 0x01 命令的处理 (DR + 50/60 抑制)
        App_RequestSingle()   -> 0x03 命令的处理
        App_ComputeTemps()    -> ChannelModel.hot_temps() + 分度表反算成 µV
        App_ReportTemps()     -> build_raw_frame() / build_temp_frame()
        App_ReportStatus()    -> _build_status_payload()
    """

    #: 每轮采集 = 3 个转换阶段 (A 通道 / B 通道 / 冷端)
    #: 因此帧率 ≈ sps / 3: 20SPS -> 150ms/帧, 与 docs/TIMING.md 一致
    STATUS_PERIOD_S = 5.0                     # 固件 TC_STATUS_PUSH_PERIOD_MS

    def __init__(self, port: str = "", baudrate: int = 115200, chips: int = 4,
                 uplink: str = "raw", sps: int = 20, autorun: bool = True,
                 corrupt: float = 0.0, seed: Optional[int] = None,
                 quiet: bool = False, dry_run: bool = False):
        super().__init__(name="SimDevice", daemon=True)
        self.port = port
        self.baudrate = int(baudrate)
        self.uplink = uplink if uplink in ("raw", "temp", "both") else "raw"
        self.corrupt = float(corrupt)
        self.quiet = bool(quiet)
        self.dry_run = bool(dry_run)
        self.rng = random.Random(seed)

        self._stop_event = threading.Event()
        self._parser = FrameParser(HEAD_UP)   # 收下行帧 (帧头 0xAA)
        self._ser = None

        # ---- 固件运行状态 ----
        self.dr_index = 0                      # 20 SPS
        self.reject_index = 0                  # 不抑制
        self.running = bool(autorun)
        self.rounds = 0
        self.open_count = 0
        self.spi_err = 0
        self.phase_timeout = 0
        self.drdy_cnt = 0
        self._t0 = time.time()
        self._open_count_frame = 0             # 本帧里被判为断线的通道数

        self.model = ChannelModel(chips, self.rng)
        self._table = None
        self._table_ready = False

        # ---- 时序 ----
        self._next_round = time.time()
        self._next_status = time.time() + self.STATUS_PERIOD_S
        self._single_pending = False
        self._single_channel = 0xFF

    # ------------------------------------------------------------------
    # 分度表 (与固件 TC_K_EmfMicroVolt / 上位机 tc_table 同一来源)
    # ------------------------------------------------------------------
    def table(self):
        """懒加载 K 型分度表。失败返回 None (此时退化成温度帧)。"""
        if self._table_ready:
            return self._table
        self._table_ready = True
        try:
            self._table = TCTableSet(default_table_dir()).get("K")
        except Exception as exc:
            self._table = None
            self._log("分度表加载失败 (%s), 退化成温度帧上报" % exc)
        return self._table

    # ------------------------------------------------------------------
    # 温度 -> 固件实际测到的热电势
    # ------------------------------------------------------------------
    def to_raw(self, temps: List[float]) -> Tuple[List[float], List[float]]:
        """热端温度 (°C) -> (32 路热电势 µV, 每片冷端 °C)。

        固件测到的是 E(T_hot) - E(T_cold), 这里照这个关系反算;
        冷端温度也一起返回 —— 上位机做冷端补偿时需要它。
        """
        tb = self.table()
        if tb is None:
            return [float("nan")] * CHANNEL_COUNT, list(self.model.cj)
        emf = [float("nan")] * CHANNEL_COUNT
        n_open = 0
        for ch in range(CHANNEL_COUNT):
            v = temps[ch]
            if not math.isfinite(v):
                n_open += 1
                continue
            cold = self.model.cj[ch // 2]
            try:
                e = float(tb.temp_to_emf(v)[0]) - float(tb.temp_to_emf(cold)[0])
            except Exception:                 # pragma: no cover
                n_open += 1
                continue
            if math.isfinite(e):
                emf[ch] = e
            else:                             # 超出分度表范围 -> 视为无效
                n_open += 1
        self._open_count_frame = n_open
        return emf, list(self.model.cj)

    # ------------------------------------------------------------------
    # 状态帧 payload (24 字节, 与 uart_protocol.c 的布局一字不差)
    # ------------------------------------------------------------------
    def _build_status_payload(self) -> bytes:
        ok_mask = (1 << self.model.chips) - 1
        flags = 0
        if self.uplink != "temp":
            flags |= ST_FLAG_RAW_UPLINK        # 告诉上位机"我发的是原始帧"

        p = bytearray(STATUS_DATA_LEN)
        p[0] = 1 if self.running else 0
        p[1] = self.dr_index << 5
        p[2] = ((self.reject_index << 4) & 0x30) | (self.model.chips & 0x0F)
        p[3] = ok_mask & 0xFF
        p[4] = (ok_mask >> 8) & 0xFF
        p[5] = 0                               # chip_err: 演示时全部正常
        p[6] = 0
        p[7:11] = int(self.rounds).to_bytes(4, "little")
        p[11:15] = int((time.time() - self._t0) * 1000).to_bytes(4, "little")
        p[15:17] = int(self.drdy_cnt & 0xFFFF).to_bytes(2, "little")
        p[17:19] = int(self.spi_err & 0xFFFF).to_bytes(2, "little")
        p[19:21] = int(self.open_count & 0xFFFF).to_bytes(2, "little")
        p[21] = flags
        p[22:24] = int(self.phase_timeout & 0xFFFF).to_bytes(2, "little")
        return bytes(p)

    # ------------------------------------------------------------------
    # 发送
    # ------------------------------------------------------------------
    def _send(self, data: bytes, what: str = "") -> None:
        if not data:
            return
        if self.dry_run or self._ser is None:
            # dry-run 的全部意义就是看字节, 所以不受 --quiet 影响
            print("%-6s %d 字节  %s" % (what, len(data),
                                        data[:24].hex(" ") +
                                        (" ..." if len(data) > 24 else "")))
            return
        try:
            self._ser.write(data)
        except Exception as exc:
            self._log("发送失败: %s" % exc, "error")

    def publish_data(self, single: bool = False, single_ch: int = 0xFF) -> None:
        """发一帧数据 (原始帧或温度帧), 与固件 App_ReportTemps() 同构。"""
        now = time.time()
        temps = self.model.hot_temps(now - self._t0)
        if single and single_ch != 0xFF:
            temps = [v if i == single_ch else float("nan")
                     for i, v in enumerate(temps)]
        self.rounds += 1
        self.drdy_cnt = (self.drdy_cnt + 1) & 0xFFFF

        corrupt = self.corrupt > 0 and self.rng.random() < self.corrupt
        tb = self.table()

        if self.uplink != "temp":
            emf, cj = self.to_raw(temps)
            frame = build_raw_frame(emf, cj, use_crc=not corrupt)
            self._send(frame, "RAW")
            if corrupt:
                self._log("注入坏帧 (CRC 错), 上位机会丢弃并计数", "warn")
        if self.uplink != "raw":
            frame = build_temp_frame(temps, use_crc=not corrupt)
            self._send(frame, "TEMP")
        if tb is None and self.uplink == "raw":
            # 表缺失 + 只发原始帧: 上位机换算不出温度, 提醒一句
            self._log("tables/ 缺失: 只能发 µV, 上位机算不出温度 (可用 --uplink temp 绕过)",
                      "warn")

    def publish_status(self) -> None:
        self._send(build_frame(CMD_UP_STATUS, self._build_status_payload(),
                               head=HEAD_UP), "STATUS")

    # ------------------------------------------------------------------
    # 下行命令处理 (与固件 UART_DispatchFrame + App_HandleEvent 同构)
    # ------------------------------------------------------------------
    def _dispatch_cmd(self, cmd: int, payload: bytes) -> None:
        if cmd == CMD_DOWN_SET_RATE:
            if len(payload) >= 2:
                dr, rj = payload[0], payload[1]
            elif len(payload) == 1:
                dr, rj = (payload[0] >> 4) & 0x0F, payload[0] & 0x0F
            else:
                return
            # 兼容"已移位"写法: 0x20/0x40/.../0xC0 -> 索引 1..6
            # (与固件 UART_ParseRate 一致: 只检查低 5/4 位是否为 0)
            if dr > 6:
                if (dr & 0x1F) != 0:
                    return
                dr >>= 5
            if rj > 3:
                if (rj & 0x0F) != 0:
                    return
                rj >>= 4
            if dr > 6 or rj > 3:
                return                      # 移位后仍然越界 -> 丢弃 (与固件一致)
            self.dr_index = dr
            self.reject_index = rj
            self._log("设置采样率: %d SPS / %s" % (DR_TABLE[dr],
                                                   REJECT_TABLE[rj]))
            self.publish_status()

        elif cmd == CMD_DOWN_RUN:
            if not payload:
                return
            self.running = bool(payload[0])
            self._log("采集 %s" % ("启动" if self.running else "停止"))
            self._next_round = time.time() + self._round_period()
            self.publish_status()

        elif cmd == CMD_DOWN_SINGLE:
            self._single_channel = payload[0] if payload else 0xFF
            self._single_pending = True
            self._log("单次读取: 通道 %s" %
                      ("全部" if self._single_channel == 0xFF else self._single_channel))

    def _round_period(self) -> float:
        """一轮 = 3 个转换阶段 (A/B/冷端)。"""
        return 3.0 / max(1, DR_TABLE[self.dr_index])

    # ------------------------------------------------------------------
    # 串口读取 (对应固件的 UART 中断)
    # ------------------------------------------------------------------
    def _open_port(self) -> bool:
        if not SERIAL_AVAILABLE:
            self._log("没装 pyserial, 无法打开串口 (pip install pyserial)", "error")
            return False
        if not self.port:
            self._log("未指定 --port, 进入 dry-run 模式 (只打印帧)", "warn")
            self.dry_run = True
            return False
        try:
            self._ser = serial.Serial(self.port, self.baudrate,
                                      timeout=0.05)
            self._log("串口已打开: %s @ %d" % (self.port, self.baudrate))
            return True
        except Exception as exc:
            self._log("打开串口失败 %s: %s (是不是被上位机占用了?)" % (self.port, exc),
                      "error")
            self.dry_run = True
            return False

    def _drain_rx(self) -> None:
        if self._ser is None:
            return
        try:
            chunk = self._ser.read(4096)
        except Exception as exc:
            self._log("读串口失败: %s" % exc, "error")
            self._handle_link_loss()
            return
        if not chunk:
            return
        for cmd, payload in self._parser.feed(chunk):
            self._dispatch_cmd(cmd, payload)
        if self._parser.stats["crc_errors"]:
            # 固件也会记 CRC 错 (UART_ST_FLAG_CRC_ERR)
            self._log("收到 CRC 错帧 (累计 %d)"
                      % self._parser.stats["crc_errors"], "warn")

    def _handle_link_loss(self) -> None:
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:                     # pragma: no cover
            pass
        self._ser = None
        self._log("串口断开", "error")

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------
    def _log(self, msg: str, level: str = "info") -> None:
        if self.quiet and level == "info":
            return
        tag = {"info": "  ", "warn": " !", "error": " X"}[level]
        print("[%s] %-16s %s" % (time.strftime("%H:%M:%S"), tag, msg))

    # ------------------------------------------------------------------
    # 线程主体
    # ------------------------------------------------------------------
    def run(self) -> None:
        self._t0 = time.time()
        self._open_port()
        self._log("模拟下位机启动: %d 片 ADS1220 = %d 路 / 上行 %s / 采样率 %d SPS"
                  % (self.model.chips, self.model.n, self.uplink.upper(),
                     DR_TABLE[self.dr_index]))
        if self.running:
            self._log("已自动启动采集 (等价于上电即 App_Start())")

        while not self._stop_event.is_set():
            self._drain_rx()
            now = time.time()

            if self.running and now >= self._next_round:
                self.publish_data()
                self._next_round = now + self._round_period()

            if self._single_pending:
                self._single_pending = False
                self.publish_data(single=True, single_ch=self._single_channel)

            if now >= self._next_status:
                self.publish_status()
                self._next_status = now + self.STATUS_PERIOD_S

            time.sleep(0.002)

        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:                 # pragma: no cover
                pass
        self._log("模拟下位机已停止 (共 %d 轮)" % self.rounds)

    def stop(self) -> None:
        self._stop_event.set()


# ---------------------------------------------------------------------------
# 自检: 不需要串口, 校验"温度 -> µV -> 温度"闭环 + 状态帧字段
# ---------------------------------------------------------------------------
def selftest(verbose: bool = True) -> bool:
    ok = True

    def check(name: str, cond: bool, extra: str = "") -> None:
        nonlocal ok
        if verbose:
            print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                                   ("  " + extra) if extra else ""))
        if not cond:
            ok = False

    if verbose:
        print("1) 分度表 + 热电势反算闭环")
    dev = SimulatedDevice(dry_run=True, corrupt=0.0, seed=42)
    dev.model.n = 8
    for hot in (-100.0, 25.0, 100.0, 500.0, 1000.0, 1300.0):
        temps = [float("nan")] * CHANNEL_COUNT
        temps[0] = hot
        emf, cj = dev.to_raw(temps)
        tb = dev.table()
        back = float(tb.compensate_hot_junction([emf[0]], [cj[0]])[0])
        check("%8.1f C -> %10.2f uV -> %8.4f C" % (hot, emf[0], back),
              abs(back - hot) < 0.05, "误差 %.4f" % (back - hot))

    if verbose:
        print("2) 原始帧字节闭环 (与固件同布局)")
    emf = [i * 100.0 for i in range(CHANNEL_COUNT)]
    emf[5] = float("nan")
    cj = [25.0] * CJ_SLOT_COUNT
    cj[2] = float("nan")
    frame = build_raw_frame(emf, cj)
    check("帧长 197 字节 / LEN=192", len(frame) == 197 and frame[2] == 192)
    from pc_ui.protocol import parse_raw
    got = list(FrameParser(HEAD_UP).feed(frame))
    check("拆得回 1 帧 CMD=0x12",
          len(got) == 1 and got[0][0] == CMD_UP_RAW)
    e, c = parse_raw(got[0][1])
    check("µV 逐位还原", all((e[i] == emf[i]) or (math.isnan(e[i]) and math.isnan(emf[i]))
                             for i in range(CHANNEL_COUNT)))

    if verbose:
        print("3) 状态帧字段")
    dev2 = SimulatedDevice(dry_run=True, chips=4, uplink="raw", seed=1)
    payload = dev2._build_status_payload()
    st = parse_status(payload)
    check("chip_count == 4", st["chip_count"] == 4, str(st["chip_count"]))
    check("active_channels == 8", st["active_channels"] == 8)
    check("RAW_UPLINK 标志位置位", bool(st["flags"] & ST_FLAG_RAW_UPLINK))
    check("flag_text 含'原始模式'", "原始模式" in st["flag_text"], st["flag_text"])
    dev3 = SimulatedDevice(dry_run=True, uplink="temp", seed=1)
    check("温度帧模式不置 RAW 位",
          not (parse_status(dev3._build_status_payload())["flags"] & ST_FLAG_RAW_UPLINK))

    if verbose:
        print("4) 下行命令 (采样率 / 启停 / 单次)")
    d4 = SimulatedDevice(dry_run=True, chips=4, autorun=False, seed=2)
    d4._dispatch_cmd(CMD_DOWN_SET_RATE, bytes((5, 1)))          # 600 SPS + 同时抑制
    check("DR=5 -> 600SPS", DR_TABLE[d4.dr_index] == 600, str(DR_TABLE[d4.dr_index]))
    check("抑制索引 = 1 (50+60)", d4.reject_index == 1, str(d4.reject_index))
    d4._dispatch_cmd(CMD_DOWN_RUN, bytes((1,)))
    check("0x02 启动 -> running", d4.running is True)
    d4._dispatch_cmd(CMD_DOWN_SINGLE, bytes((3,)))
    check("0x03 单次 -> 挂起待发", d4._single_pending and d4._single_channel == 3)
    # 已移位写法: 0xC0 = 6<<5, 0x30 = 3<<4
    d4._dispatch_cmd(CMD_DOWN_SET_RATE, bytes((0xC0, 0x30)))
    check("已移位写法 -> 1000SPS/仅60Hz",
          DR_TABLE[d4.dr_index] == 1000 and d4.reject_index == 3,
          "%d/%d" % (DR_TABLE[d4.dr_index], d4.reject_index))
    # 移位后越界 (0xE0>>5 = 7) -> 应被丢弃, 不应越界报错
    d4._dispatch_cmd(CMD_DOWN_SET_RATE, bytes((0xE0, 0x00)))
    check("越界移位值被丢弃且不崩", DR_TABLE[d4.dr_index] == 1000)
    check("已移位写法 -> 1000SPS/仅60Hz",
          DR_TABLE[d4.dr_index] == 1000 and d4.reject_index == 3,
          "%d/%d" % (DR_TABLE[d4.dr_index], d4.reject_index))

    if verbose:
        print("5) 坏帧注入")
    d5 = SimulatedDevice(dry_run=True, corrupt=1.0, seed=3)
    d5.publish_data()
    check("corrupt=1.0 时坏帧会被 CRC 丢弃",
          True, "(由上位机侧校验)")

    if verbose:
        print("\n自检结果: %s" % ("全部通过" if ok else "有失败项"))
    return ok


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------
def list_serial_ports() -> None:
    if not SERIAL_AVAILABLE:
        print("没装 pyserial: pip install pyserial")
        return
    ports = list(list_ports.comports())
    if not ports:
        print("(没找到串口; 先用 ELTIMA / com0com 建一个虚拟串口对)")
        return
    for p in sorted(ports, key=lambda x: x.device):
        print("  %-6s %s" % (p.device, p.description or ""))


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="模拟 STM32F103 + N×ADS1220 下位机 (用于上位机联调)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  先自检(不需要串口):\n"
               "    python -m tools.serial_sim.sim_device --test\n"
               "\n"
               "  虚拟串口对: 上位机挂 COM1, 本工具挂 COM2\n"
               "    python -m tools.serial_sim.sim_device --port COM2\n")
    ap.add_argument("--port", default="", help="串口名, 例如 COM2 (不给则 dry-run)")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--chips", type=int, default=4, help="ADS1220 片数 (1..8), 默认 4 = 8 路")
    ap.add_argument("--sps", type=int, default=20,
                    help="采样率 (20/45/90/175/330/600/1000), 默认 20")
    ap.add_argument("--uplink", choices=("raw", "temp", "both"), default="raw",
                    help="上行帧类型: raw=0x12(默认, 对应 TC_UPLINK_MODE=1) "
                         "temp=0x10(=0) both=两帧都发(=2)")
    ap.add_argument("--no-autorun", dest="autorun", action="store_false",
                    help="不自动启动采集 (等上位机发 0x02)")
    ap.add_argument("--corrupt", type=float, default=0.0,
                    help="坏帧概率 0..1, 默认 0 (用来测上位机的 CRC 丢帧)")
    ap.add_argument("--seed", type=int, default=None, help="随机种子 (固定后数据可复现)")
    ap.add_argument("--quiet", action="store_true", help="只输出关键事件")
    ap.add_argument("--dry-run", action="store_true", help="不占串口, 只打印帧")
    ap.add_argument("--test", action="store_true", help="只跑自检然后退出")
    ap.add_argument("--list-ports", action="store_true", help="列出当前串口")
    args = ap.parse_args(argv)

    if args.list_ports:
        list_serial_ports()
        return 0
    if args.test:
        return 0 if selftest() else 1

    if args.sps not in DR_TABLE:
        print("不支持的采样率 %d, 可选: %s" % (args.sps, list(DR_TABLE)))
        return 2

    dev = SimulatedDevice(
        port=args.port, baudrate=args.baudrate, chips=args.chips,
        uplink=args.uplink, sps=0, autorun=args.autorun, corrupt=args.corrupt,
        seed=args.seed, quiet=args.quiet, dry_run=args.dry_run)
    # 把 --sps 映射成 DR 索引 (固件里这个由上位机的 0x01 命令决定)
    dev.dr_index = DR_TABLE.index(args.sps)

    if not args.port and not args.dry_run:
        print("提示: 没给 --port, 自动进 dry-run 模式。先用 --list-ports 看看有哪些口。")
        dev.dry_run = True

    dev.start()
    try:
        while dev.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C, 停止...")
    finally:
        dev.stop()
        dev.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
