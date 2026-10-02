#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
32 路热电偶测温系统 —— 上位机通信协议层 (纯逻辑, 不依赖 GUI / 串口)

本文件是固件 Core/Inc/uart_protocol.h + Core/Src/uart_protocol.c 的 Python 对偶实现,
两边共用同一套 CRC / 组帧 / 拆帧规则, 可以直接互相验证。

========================== 帧格式 ==========================
上位机 -> STM32 (下行命令):
    +------+------+------+------------------+--------+--------+
    | 0xAA | CMD  | LEN  | DATA[0..LEN-1]   | CRC_LO | CRC_HI |
    +------+------+------+------------------+--------+--------+
STM32 -> 上位机 (上行上报):
    +------+------+------+------------------+--------+--------+
    | 0x55 | CMD  | LEN  | DATA[0..LEN-1]   | CRC_LO | CRC_HI |
    +------+------+------+------------------+--------+--------+

  LEN  = DATA 段字节数 (不含 CMD/LEN/CRC)
  CRC  = CRC-16/MODBUS (poly 0xA001 反射, init 0xFFFF), 覆盖帧头到最后一个 DATA 字节,
         低字节在前。标准向量: b"123456789" -> 0x4B37

上行 CMD:
  0x10 温度: DATA = 32 x float32 小端 (°C), 顺序 ch[2n]=第 n 片 AIN0/AIN1,
             ch[2n+1]=第 n 片 AIN2/AIN3。无效通道 = quiet NaN (断线/通信失败)。
  0x11 状态: DATA = 24 字节, 见 parse_status()

下行 CMD:
  0x01 设置采样率: LEN=2 -> DATA = [DR索引(0..6), 抑制索引(0..3)]
  0x02 启动/停止:  DATA = [0 停止 / 1 启动]
  0x03 单次读取:   DATA 省略 或 [通道号 0..31 / 0xFF=全部]

自检:
    python -m pc_ui.protocol
"""

from __future__ import annotations

import math
import struct
import sys
import time
from typing import Dict, Iterator, List, Optional, Tuple

# ---------------------------------------------------------------------------
# 协议常量 (必须与 Core/Inc/uart_protocol.h 保持一致)
# ---------------------------------------------------------------------------
HEAD_DOWN = 0xAA                      # PC -> MCU 帧头
HEAD_UP = 0x55                        # MCU -> PC 帧头

CMD_UP_TEMP = 0x10                    # 上行: 32 路温度
CMD_UP_STATUS = 0x11                  # 上行: 状态
CMD_DOWN_SET_RATE = 0x01              # 下行: 设置采样率
CMD_DOWN_RUN = 0x02                   # 下行: 启动/停止采集
CMD_DOWN_SINGLE = 0x03                # 下行: 读取单次温度

CHANNEL_COUNT = 32                    # 总测温通道数
CHIP_COUNT = 16                       # 16 x ADS1220, 每片 2 路差分
STATUS_DATA_LEN = 24                  # 状态帧 DATA 长度
TEMP_DATA_LEN = CHANNEL_COUNT * 4     # 温度帧 DATA 长度 = 128
MAX_DATA_LEN = 200                    # 固件限制的 DATA 上限
CH_SINGLE_ALL = 0xFF                  # CMD=0x03 时表示"全部通道"

#: 帧内字节间超时 (秒)。超过则认为半截帧作废, 重新找帧头 (与固件 50ms 一致)
FRAME_GAP_S = 0.050

#: DR 索引 -> SPS
DR_TABLE: Tuple[int, ...] = (20, 45, 90, 175, 330, 600, 1000)
#: 界面上允许用户选择的采样率 (需求: 20/45/90/175/330/600 SPS)
SAMPLE_RATE_CHOICES: Tuple[int, ...] = (20, 45, 90, 175, 330, 600)
#: 50/60Hz 抑制索引 -> 文本
REJECT_TABLE: Tuple[str, ...] = ("不抑制", "同时抑制50+60Hz", "仅抑制50Hz", "仅抑制60Hz")
REJECT_OFF, REJECT_BOTH, REJECT_50, REJECT_60 = 0, 1, 2, 3

#: 状态帧 flags 位
ST_FLAG_SELFTEST = 0x01               # 上电自检失败
ST_FLAG_DRDY_SILENT = 0x02            # 长时间收不到 DRDY
ST_FLAG_VERIFY_FAIL = 0x04            # 上电回读校验失败
ST_FLAG_TX_OVERFLOW = 0x08            # 串口发送缓冲溢出
ST_FLAG_CRC_ERR = 0x10                # 收到过 CRC 错的下行帧
ST_FLAG_DRDY_PARTIAL = 0x20           # 有片一直不就绪 (看 chip_err 位图定位)

ST_FLAG_TEXT = (
    (ST_FLAG_SELFTEST, "自检失败"),
    (ST_FLAG_DRDY_SILENT, "DRDY静默"),
    (ST_FLAG_VERIFY_FAIL, "回读校验失败"),
    (ST_FLAG_TX_OVERFLOW, "发送缓冲溢出"),
    (ST_FLAG_CRC_ERR, "收到CRC错帧"),
    (ST_FLAG_DRDY_PARTIAL, "有片无DRDY"),
)

#: 温度帧的**固定**槽位数 —— 与板子实际通道数无关。
#: 4 路板 / 8 路板 / 16 路板 共用同一套协议与同一份上位机,
#: 没接的槽位填 NaN; 真实通道数由状态帧的 chip_count 告知。
TEMP_SLOT_COUNT = 32

#: K 型热电偶物理量程 (°C), 来自 Core/Inc/thermocouple.h
TC_TEMP_MIN_C = -200.0
TC_TEMP_MAX_C = 1372.0


# ---------------------------------------------------------------------------
# CRC-16/MODBUS
# ---------------------------------------------------------------------------
def _build_crc_table() -> Tuple[int, ...]:
    """预生成 256 项查表, 逐位算法等价的加速版本。"""
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
        table.append(crc & 0xFFFF)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS: 多项式 0xA001(0x8005 反射), 初值 0xFFFF, 输入输出反射。

    >>> hex(crc16_modbus(b"123456789"))
    '0x4b37'
    """
    crc = 0xFFFF
    table = _CRC_TABLE
    for byte in data:
        crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def crc16_modbus_bitwise(data: bytes) -> int:
    """逐位实现 (慢, 仅用于自检对照)。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# 组帧
# ---------------------------------------------------------------------------
def build_frame(cmd: int, data: bytes = b"", head: int = HEAD_DOWN,
                use_crc: bool = True) -> bytes:
    """组一帧: HEAD CMD LEN DATA CRC_LO CRC_HI。

    :param cmd:      命令字
    :param data:     DATA 段 (<= 200 字节)
    :param head:     帧头, 下行 0xAA / 上行 0x55
    :param use_crc:  False 时故意不发正确 CRC (仅用于故障注入测试)
    """
    if not 0 <= cmd <= 0xFF:
        raise ValueError("cmd 必须是 0..255")
    if len(data) > MAX_DATA_LEN:
        raise ValueError("DATA 超过 %d 字节" % MAX_DATA_LEN)
    body = bytes((head, cmd, len(data))) + bytes(data)
    if not use_crc:
        return body + b"\x00\x00"
    crc = crc16_modbus(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def build_temp_frame(temps: List[float], head: int = HEAD_UP,
                     use_crc: bool = True) -> bytes:
    """按 CMD=0x10 组一帧 32 路温度 (供模拟器/测试使用)。"""
    values = list(temps)[:CHANNEL_COUNT]
    while len(values) < CHANNEL_COUNT:
        values.append(float("nan"))
    payload = struct.pack("<%df" % CHANNEL_COUNT, *values)
    return build_frame(CMD_UP_TEMP, payload, head=head, use_crc=use_crc)


def build_status_frame(payload: bytes, head: int = HEAD_UP) -> bytes:
    """按 CMD=0x11 组一帧状态。"""
    return build_frame(CMD_UP_STATUS, payload, head=head)


# ---------------------------------------------------------------------------
# 拆帧状态机 (与固件 UART_Protocol_OnRxByte 一字对应)
# ---------------------------------------------------------------------------
class FrameParser:
    """只认帧头 + LEN + CRC, 丢字节/混入噪声后能自动重新同步。

    使用方式::

        p = FrameParser(HEAD_UP)
        for cmd, payload in p.feed(chunk):
            ...

    统计信息放在 :attr:`stats` 里 (CRC 错帧数 / 重新同步次数), 供 UI 显示。
    """

    def __init__(self, head: int = HEAD_UP):
        self.head = head
        self.stats = {
            "frames_ok": 0,      # 校验通过并交付的帧数
            "crc_errors": 0,     # CRC 校验失败被丢弃的帧数
            "len_errors": 0,     # LEN 非法导致的重同步
            "bytes": 0,          # 累计喂入字节数
        }
        self._reset()

    # -- 内部状态 ---------------------------------------------------------
    def _reset(self) -> None:
        self.buf = bytearray()
        self.need = 0
        self.state = "HEAD"

    def reset(self) -> None:
        """丢弃当前半截帧, 重新找帧头 (帧内字节间超时时调用)。"""
        self._reset()

    # -- 喂数据 -----------------------------------------------------------
    def feed(self, data: bytes) -> Iterator[Tuple[int, bytes]]:
        """喂入任意长度字节流, 逐个产出校验通过的 (cmd, payload)。"""
        self.stats["bytes"] += len(data)
        for byte in data:
            frame = self._feed_byte(byte)
            if frame is not None:
                yield frame

    def _feed_byte(self, byte: int) -> Optional[Tuple[int, bytes]]:
        state = self.state

        if state == "HEAD":
            if byte == self.head:
                self.buf = bytearray((byte,))
                self.state = "CMD"
            return None

        if state == "CMD":
            self.buf.append(byte)
            self.state = "LEN"
            return None

        if state == "LEN":
            if byte > MAX_DATA_LEN:
                # 非法长度: 这一帧作废, 重新找帧头
                self.stats["len_errors"] += 1
                self._reset()
                return None
            self.buf.append(byte)
            self.need = byte
            self.state = "CRC_LO" if byte == 0 else "DATA"
            return None

        if state == "DATA":
            self.buf.append(byte)
            self.need -= 1
            if self.need == 0:
                self.state = "CRC_LO"
            return None

        if state == "CRC_LO":
            self.buf.append(byte)
            self.state = "CRC_HI"
            return None

        if state == "CRC_HI":
            self.buf.append(byte)
            calc = crc16_modbus(bytes(self.buf[:-2]))
            recv = self.buf[-2] | (self.buf[-1] << 8)
            cmd = self.buf[1]
            payload = bytes(self.buf[3:-2])
            self._reset()
            if calc == recv:
                self.stats["frames_ok"] += 1
                return (cmd, payload)
            # CRC 失败 -> 整帧丢弃 (绝不输出错误数据), 等下一个帧头
            self.stats["crc_errors"] += 1
            return None

        self._reset()
        return None


# ---------------------------------------------------------------------------
# 上行帧解析
# ---------------------------------------------------------------------------
def parse_temp(payload: bytes) -> List[float]:
    """CMD=0x10 -> 32 个温度 (float, 无效通道为 nan)。

    容错: DATA 长度不是 4 的整数倍时, 只解析完整的部分并补 nan 到 32 路,
    保证 UI 拿到的永远是定长列表。
    """
    n = min(len(payload) // 4, CHANNEL_COUNT)
    values = list(struct.unpack("<%df" % n, payload[: n * 4])) if n else []
    if len(values) < CHANNEL_COUNT:
        values.extend([float("nan")] * (CHANNEL_COUNT - len(values)))
    # 固件用 0x7FC00000 表示无效; struct 解出来就是 nan, 但防御性地把 inf 也变 nan
    return [v if math.isfinite(v) else float("nan") for v in values]


def parse_status(payload: bytes) -> Dict:
    """CMD=0x11 -> dict (字段含义见 Core/Inc/uart_protocol.h)。"""
    if len(payload) < STATUS_DATA_LEN:
        return {"error": "状态帧长度不足: %d < %d" % (len(payload), STATUS_DATA_LEN)}

    d = payload
    dr_bits = d[1] & 0xE0
    dr_sps = None
    for index, sps in enumerate(DR_TABLE):
        if (index << 5) == dr_bits:
            dr_sps = sps
            break

    flags = d[21]
    flag_list = [text for bit, text in ST_FLAG_TEXT if flags & bit]
    chip_ok = d[3] | (d[4] << 8)
    chip_err = d[5] | (d[6] << 8)

    # ★ 本板实际片数: 固件放在状态帧 byte2 的低 4 位 (高 4 位仍是 50/60 抑制)。
    #   老固件该字段恒为 0, 这时退化成"按 OK|ERR 位图推断", 再不行就按 32 槽。
    chip_count = d[2] & 0x0F
    if chip_count:
        active_channels = min(chip_count * 2, TEMP_SLOT_COUNT)
    else:
        mask = chip_ok | chip_err
        inferred = mask.bit_length()          # 最高置位 bit + 1 = 扫过的片数
        active_channels = min(inferred * 2, TEMP_SLOT_COUNT) if inferred else TEMP_SLOT_COUNT

    return {
        "run": d[0],
        "dr_bits": dr_bits,
        "dr_sps": dr_sps,
        "reject_bits": d[2] & 0x30,
        "reject": REJECT_TABLE[(d[2] >> 4) & 0x03],
        "chip_count": chip_count,
        "active_channels": active_channels,
        "chip_ok": chip_ok,
        "chip_err": chip_err,
        # bit n = 1 表示第 n 片正常
        "chip_ok_text": "".join("1" if (chip_ok >> i) & 1 else "0" for i in range(CHIP_COUNT - 1, -1, -1)),
        "chip_err_text": "".join("1" if (chip_err >> i) & 1 else "0" for i in range(CHIP_COUNT - 1, -1, -1)),
        "rounds": int.from_bytes(d[7:11], "little"),
        "uptime_ms": int.from_bytes(d[11:15], "little"),
        "drdy_cnt": d[15] | (d[16] << 8),
        "spi_err": d[17] | (d[18] << 8),
        "open_cnt": d[19] | (d[20] << 8),
        "timeout_cnt": d[22] | (d[23] << 8),
        "flags": flags,
        "flag_text": ",".join(flag_list) if flag_list else "正常",
        "raw": bytes(payload),
    }


# ---------------------------------------------------------------------------
# 下行命令构造
# ---------------------------------------------------------------------------
def dr_index_for_sps(sps: int) -> int:
    """采样率 (SPS) -> DR 索引 0..6; 不在表里就抛异常。"""
    try:
        return DR_TABLE.index(int(sps))
    except ValueError:
        raise ValueError("不支持的采样率 %r, 可选: %s" % (sps, list(DR_TABLE))) from None


def cmd_set_rate(dr_index: int, reject_index: int = REJECT_OFF) -> bytes:
    """CMD=0x01 设置采样率: DATA = [DR索引, 抑制索引]。

    :param dr_index:     0..6  (0=20SPS 1=45 2=90 3=175 4=330 5=600 6=1000)
    :param reject_index: 0..3  (0=不抑制 1=50+60 2=仅50 3=仅60; 只能配合 20SPS)
    """
    if not 0 <= dr_index <= 6:
        raise ValueError("DR 索引必须是 0..6")
    if not 0 <= reject_index <= 3:
        raise ValueError("抑制索引必须是 0..3")
    return build_frame(CMD_DOWN_SET_RATE, bytes((dr_index, reject_index)))


def cmd_set_rate_sps(sps: int, reject_index: int = REJECT_OFF) -> bytes:
    """CMD=0x01 的便利版本: 直接传 SPS。"""
    return cmd_set_rate(dr_index_for_sps(sps), reject_index)


def cmd_run(start: bool) -> bytes:
    """CMD=0x02 启动/停止采集。"""
    return build_frame(CMD_DOWN_RUN, bytes((1 if start else 0,)))


def cmd_single(channel: int = CH_SINGLE_ALL) -> bytes:
    """CMD=0x03 读取单次温度。channel=0..31 或 0xFF(全部)。"""
    if channel != CH_SINGLE_ALL and not 0 <= channel < CHANNEL_COUNT:
        raise ValueError("通道号必须是 0..31 或 0xFF")
    return build_frame(CMD_DOWN_SINGLE, bytes((channel,)))


# ---------------------------------------------------------------------------
# 采样率 -> 波特率建议 (来自 docs/TIMING.md: 高速采样率必须用 460800)
# ---------------------------------------------------------------------------
def baudrate_advice(sps: int) -> Optional[str]:
    """返回一条警告文本 (无问题时返回 None)。

    一帧温度 = 133 字节; 115200bps 下一帧约 11.5ms, 若帧率高于此值会积压丢帧。
    """
    if sps >= 175:
        return "采样率 %d SPS 建议使用 460800 波特率 (115200 下 133 字节/帧约 11.5ms)" % sps
    return None


# ---------------------------------------------------------------------------
# 自检
# ---------------------------------------------------------------------------
def selftest(verbose: bool = True) -> bool:
    """CRC + 组帧/拆帧闭环 + 抗干扰 + 状态解析自检。"""
    ok = True

    def check(name: str, cond: bool) -> None:
        nonlocal ok
        if verbose:
            print("  [%s] %s" % ("OK" if cond else "FAIL", name))
        if not cond:
            ok = False

    if verbose:
        print("1) CRC-16/MODBUS")
    check('crc16(b"123456789") == 0x4B37', crc16_modbus(b"123456789") == 0x4B37)
    check("查表版 == 逐位版",
          all(crc16_modbus(bytes((i, j))) == crc16_modbus_bitwise(bytes((i, j)))
              for i in range(0, 256, 17) for j in range(0, 256, 31)))

    if verbose:
        print("2) 下行帧构造 -> 拆帧闭环")
    for name, frame, want_cmd in (
        ("set_rate 20SPS/抑制50+60", cmd_set_rate(0, REJECT_BOTH), CMD_DOWN_SET_RATE),
        ("set_rate 600SPS", cmd_set_rate_sps(600), CMD_DOWN_SET_RATE),
        ("start", cmd_run(True), CMD_DOWN_RUN),
        ("stop", cmd_run(False), CMD_DOWN_RUN),
        ("single 全部", cmd_single(), CMD_DOWN_SINGLE),
        ("single ch5", cmd_single(5), CMD_DOWN_SINGLE),
    ):
        got = list(FrameParser(HEAD_DOWN).feed(frame))
        check("%-24s -> %s" % (name, got), len(got) == 1 and got[0][0] == want_cmd)

    if verbose:
        print("3) 上行温度帧闭环 (含 NaN)")
    temps = [i * 1.5 for i in range(CHANNEL_COUNT)]
    temps[7] = float("nan")
    frame = build_temp_frame(temps)
    check("帧长 = 3 + 128 + 2 = 133", len(frame) == 133)
    got = list(FrameParser(HEAD_UP).feed(frame))
    check("解析出 1 帧 CMD=0x10", len(got) == 1 and got[0][0] == CMD_UP_TEMP)
    if got:
        back = parse_temp(got[0][1])
        check("长度补齐到 32", len(back) == CHANNEL_COUNT)
        check("通道 7 是 NaN", math.isnan(back[7]))
        check("通道 6 数值正确", abs(back[6] - 9.0) < 1e-6)

    if verbose:
        print("4) 抗干扰能力")
    got = list(FrameParser(HEAD_UP).feed(b"\x01\x02\x03" + frame))
    check("前导垃圾字节被跳过", len(got) == 1)
    bad = bytearray(frame)
    bad[10] ^= 0xFF
    parser = FrameParser(HEAD_UP)
    got = list(parser.feed(bytes(bad) + frame))
    check("坏帧被 CRC 丢弃, 紧随的好帧正常解析",
          len(got) == 1 and parser.stats["crc_errors"] == 1)
    check("非法 LEN 会重同步且不崩",
          list(FrameParser(HEAD_UP).feed(b"\x55\x10\xFF\x00\x01")) == [])

    if verbose:
        print("5) 状态帧解析")
    payload = bytearray(STATUS_DATA_LEN)
    payload[0] = 1             # run
    payload[1] = 0x00          # DR = 20SPS
    payload[2] = 0x14          # 高4位 抑制 = 50/60, 低4位 = 本板 4 片
    payload[3] = payload[4] = 0xFF
    payload[7:11] = (1234).to_bytes(4, "little")
    payload[21] = ST_FLAG_CRC_ERR
    st = parse_status(bytes(payload))
    check("rounds == 1234", st["rounds"] == 1234)
    check("dr_sps == 20", st["dr_sps"] == 20)
    check("chip_ok == 0xFFFF", st["chip_ok"] == 0xFFFF)
    check("flags 文本 = 收到CRC错帧", st["flag_text"] == "收到CRC错帧")
    check("短包返回 error", "error" in parse_status(b"\x00\x01"))
    # ★ 8 路版本新增字段
    check("chip_count == 4 (本板片数)", st["chip_count"] == 4)
    check("active_channels == 8 (片数x2)", st["active_channels"] == 8)
    check("抑制字段仍能正确解析", st["reject"] == "同时抑制50+60Hz")
    payload[2] = 0x00          # 模拟老固件: 没有片数字段
    old = parse_status(bytes(payload))
    check("老固件回退: 按位图推断出 32 槽", old["active_channels"] == TEMP_SLOT_COUNT)

    if verbose:
        print("6) 参数校验")
    for bad_call in (lambda: cmd_single(32), lambda: cmd_set_rate(7, 0),
                     lambda: cmd_set_rate(0, 4), lambda: dr_index_for_sps(123)):
        try:
            bad_call()
            check("非法参数应抛异常", False)
        except ValueError:
            check("非法参数抛 ValueError", True)

    if verbose:
        print("\n自检结果: %s" % ("全部通过" if ok else "有失败项"))
    return ok


if __name__ == "__main__":
    sys.exit(0 if selftest() else 1)
