"""N×ADS1220 热电偶测温系统 (本板 4 片 / 8 路) —— 上位机 (PC) 侧工具包。

* :mod:`pc_ui.protocol`       协议层: CRC / 组帧 / 拆帧 / 解析 / 下行命令构造
* :mod:`pc_ui.serial_reader`  串口后台线程: 独占串口, 收帧进 queue, 断线自动重连

本包是固件 ``Core/Inc/uart_protocol.h`` + ``Core/Src/uart_protocol.c`` 的
Python 对偶实现, 两边共用同一套 CRC / 组帧 / 拆帧规则, 可以互相验证::

    python -m pc_ui.protocol          # 协议层自检 (不需要硬件)
    python host_parser.py --test      # 同一套自检 + 命令行收发工具
    python host_parser.py COM3        # 实时打印温度
"""
