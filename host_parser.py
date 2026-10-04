#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
N x ADS1220 / 2N 路热电偶测温系统 (本板 4 片 / 8 路) —— 命令行上位机工具

协议实现只有一处: **pc_ui/protocol.py**(它同时被 pc_ui/serial_reader.py 使用),
本文件只是它的命令行外壳, 不再重复一份 CRC / 组帧逻辑 —— 避免两边算法漂移。

帧格式 / 命令字 / 状态帧布局 详见 pc_ui/protocol.py 的文件头注释,
或固件侧的 Core/Inc/uart_protocol.h。

用法:
    python host_parser.py --test                 # 协议自检 (不需要硬件)
    python host_parser.py COM3                   # 读串口并实时打印温度
    python host_parser.py /dev/ttyUSB0 -b 460800
    python host_parser.py COM3 --set-rate 0 1    # 20SPS + 同时抑制 50/60Hz
    python host_parser.py COM3 --stop
    python host_parser.py COM3 --start
    python host_parser.py COM3 --single 5        # 单次读第 5 通道
    python host_parser.py COM3 --duration 10     # 收 10 秒后退出

注意: 需要在仓库根目录下运行 (要 import pc_ui 包)。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from pc_ui.protocol import (
        CH_SINGLE_ALL,
        CHANNEL_COUNT,
        CMD_UP_STATUS,
        CMD_UP_TEMP,
        DR_TABLE,
        HEAD_UP,
        REJECT_TABLE,
        STATUS_DATA_LEN,
        FrameParser,
        cmd_run,
        cmd_set_rate,
        cmd_single,
        parse_status,
        parse_temp,
        selftest,
    )
except ImportError as exc:                       # pragma: no cover
    sys.stderr.write(
        "无法导入 pc_ui.protocol (%s)。\n"
        "请在仓库根目录下运行, 例如:  python host_parser.py --test\n" % exc
    )
    sys.exit(2)


# ---------------------------------------------------------------------------
# 打印
# ---------------------------------------------------------------------------
def format_temp_line(temps):
    """4 列一行, 同时显示 ch 序号和所属片号 / A|B 通道。"""
    parts = []
    for i, t in enumerate(temps):
        chip = i // 2
        ch = "A" if (i % 2) == 0 else "B"
        if isinstance(t, float) and math.isnan(t):
            parts.append("ch%02d(#%02d%s)=  ----  " % (i, chip, ch))
        else:
            parts.append("ch%02d(#%02d%s)=%8.2f" % (i, chip, ch, t))
        if (i % 4) == 3:
            parts.append("\n")
    return "".join(parts).rstrip("\n")


def format_status(st):
    if "error" in st:
        return "STATUS ERROR: %s" % st["error"]
    return (
        "STATUS run=%d dr=%sSPS reject=%s rounds=%d uptime=%.1fs\n"
        "       chip_ok =%s (bit15..bit0)\n"
        "       chip_err=%s\n"
        "       drdy_cnt=%d spi_err=%d timeout=%d open_tc=%d\n"
        "       flags=0x%02X [%s]"
        % (
            st["run"],
            st["dr_sps"],
            st["reject"],
            st["rounds"],
            st["uptime_ms"] / 1000.0,
            st["chip_ok_text"],
            st["chip_err_text"],
            st["drdy_cnt"],
            st["spi_err"],
            st["timeout_cnt"],
            st["open_cnt"],
            st["flags"],
            st["flag_text"],
        )
    )


# ---------------------------------------------------------------------------
# 串口实时读取 (直接用 pyserial, 不依赖 GUI 线程)
# ---------------------------------------------------------------------------
def run_serial(port, baud, commands, duration=None):
    try:
        import serial
    except ImportError:
        sys.stderr.write("需要 pyserial: pip install pyserial\n")
        return 1

    parser = FrameParser(HEAD_UP)
    t0 = time.time()

    with serial.Serial(port, baud, timeout=0.2) as ser:
        for frame in commands:
            ser.write(frame)
            ser.flush()
            print(">> 已发送: %s" % frame.hex(" ").upper())
            time.sleep(0.2)

        print(">> 开始接收 (Ctrl+C 退出)")
        while True:
            chunk = ser.read(512)
            if chunk:
                for cmd, payload in parser.feed(chunk):
                    stamp = time.strftime("%H:%M:%S")
                    if cmd == CMD_UP_TEMP:
                        temps = parse_temp(payload)
                        valid = [t for t in temps if not math.isnan(t)]
                        print("\n[%s] TEMP  %d/%d 路有效  范围 %s ~ %s C"
                              % (stamp, len(valid), len(temps),
                                 "%.2f" % min(valid) if valid else "----",
                                 "%.2f" % max(valid) if valid else "----"))
                        print(format_temp_line(temps))
                    elif cmd == CMD_UP_STATUS:
                        print("\n[%s] %s" % (stamp, format_status(parse_status(payload))))
                    else:
                        print("\n[%s] 未知上行 CMD=0x%02X LEN=%d"
                              % (stamp, cmd, len(payload)))

            if duration is not None and (time.time() - t0) > duration:
                break

    return 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="N×ADS1220 热电偶 上位机命令行工具 (本板 4 片 / 8 路)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="DR 索引: " + "  ".join("%d=%dSPS" % (i, s) for i, s in enumerate(DR_TABLE)) +
               "\n抑制索引: " + "  ".join("%d=%s" % (i, s) for i, s in enumerate(REJECT_TABLE)),
    )
    ap.add_argument("port", nargs="?", help="串口, 如 COM3 或 /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="波特率 (默认 115200; 采样率 >=175SPS 时建议 460800)")
    ap.add_argument("--test", action="store_true", help="运行协议自检 (不需要硬件)")
    ap.add_argument("--set-rate", nargs=2, type=int, metavar=("DR", "REJECT"),
                    help="设置采样率: DR 索引 0..6, 抑制索引 0..3")
    ap.add_argument("--start", action="store_true", help="启动采集")
    ap.add_argument("--stop", action="store_true", help="停止采集")
    ap.add_argument("--single", type=int, metavar="CH", nargs="?", const=CH_SINGLE_ALL,
                    help="读取单次温度 (CH=0..31 为协议槽位; 本板 0..7 有效, 省略=全部)")
    ap.add_argument("--duration", type=float, default=None, help="接收秒数后自动退出")

    args = ap.parse_args()

    if args.test or not args.port:
        return 0 if selftest() else 1

    commands = []
    if args.set_rate:
        commands.append(cmd_set_rate(args.set_rate[0], args.set_rate[1]))
    if args.start:
        commands.append(cmd_run(True))
    if args.stop:
        commands.append(cmd_run(False))
    if args.single is not None:
        commands.append(cmd_single(args.single))

    return run_serial(args.port, args.baud, commands, args.duration)


if __name__ == "__main__":
    sys.exit(main())
