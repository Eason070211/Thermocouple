#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
命令行入口 —— 32 路热电偶测温上位机

用法示例
--------
    python pc_ui/main.py --demo                     # 演示模式 (无硬件也能看界面)
    python pc_ui/main.py --demo --samples 2000      # 演示 + 保留 2000 点
    python pc_ui/main.py -p COM3                    # 连接 COM3 @115200
    python pc_ui/main.py -p COM3 -b 460800          # 连接 COM3 @460800
    python pc_ui/main.py --list-ports               # 只列出串口
    python pc_ui/main.py --selftest                 # 协议/记录模块自检 (不开窗口)
    python -m pc_ui --demo                          # 以模块方式运行
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

try:                                  # 包内导入 (python -m pc_ui.main)
    from .protocol import SAMPLE_RATE_CHOICES, selftest as protocol_selftest
    from .serial_reader import SERIAL_AVAILABLE, describe_ports
except ImportError:                   # 直接跑脚本 (python pc_ui/main.py)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from protocol import SAMPLE_RATE_CHOICES, selftest as protocol_selftest
    from serial_reader import SERIAL_AVAILABLE, describe_ports


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="tc-monitor",
        description="32 路热电偶测温上位机 (STM32F103 + 16×ADS1220, 协议 55/AA + CRC16/MODBUS)",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-p", "--port", help="串口名, 如 COM3 / /dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        choices=[115200, 460800], help="波特率 (默认 115200)")
    parser.add_argument("--demo", action="store_true",
                        help="演示模式: 用虚拟下位机产生数据 (无需硬件)")
    parser.add_argument("--samples", type=int, default=1000,
                        help="曲线保留的点数 (默认 1000)")
    parser.add_argument("--channels", type=int, default=0,
                        choices=[0, 4, 8, 16, 32],
                        help="强制本板通道数 (4/8/16/32); 默认 0 = 自动跟随固件上报的片数")
    parser.add_argument("--data-dir", default="./data",
                        help="CSV 默认保存目录 (默认 ./data)")
    parser.add_argument("--over-temp", type=float, default=1000.0,
                        help="超温告警门限 °C (默认 1000)")
    parser.add_argument("--no-auto-connect", action="store_true",
                        help="启动后不自动连接 (即使指定了 --port/--demo)")
    parser.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")
    parser.add_argument("--selftest", action="store_true",
                        help="运行协议自检后退出 (不打开界面)")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.selftest:
        return 0 if protocol_selftest() else 1

    if args.list_ports:
        ports = describe_ports()
        if not ports:
            print("未发现串口。%s" % ("" if SERIAL_AVAILABLE else "(未安装 pyserial)"))
        for device, text in ports.items():
            print("  %s" % text)
        return 0

    try:
        import tkinter  # noqa: F401
        import matplotlib  # noqa: F401
        import pandas  # noqa: F401
        import serial  # noqa: F401
    except ImportError as exc:
        print("缺少依赖: %s\n请先执行: pip install -r requirements.txt" % exc,
              file=sys.stderr)
        return 2

    try:                              # 包内导入 (python -m pc_ui.main)
        from .app import TemperatureApp
    except ImportError:               # 直接跑脚本 (python pc_ui/main.py)
        from app import TemperatureApp

    app = TemperatureApp(demo=args.demo,
                         port=args.port,
                         baudrate=args.baud,
                         retention=args.samples,
                         channels=args.channels,
                         record_dir=args.data_dir,
                         over_temp=args.over_temp,
                         auto_connect=not args.no_auto_connect)
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
