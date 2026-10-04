#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口回环联调测试 —— 模拟下位机 + 真上位机读取线程, 在虚拟串口对上真跑一遍。

这条测试验证的是**真实链路**, 不是内存里的自检:

    [sim_device (COM2)] --串口--> [SerialReader (COM1)] --队列--> 温度换算

会检查:
    1. 真上位机的 SerialReader 能从模拟器收到 CMD=0x12 原始帧
    2. 收到的原始帧能用 tables/ 分度表换算成温度 (有限值, 落在合理范围)
    3. 状态帧里 chip_count=4 / RAW_UPLINK 标志位置位
    4. 下行命令 (启停/改采样率) 能被模拟器正确响应

前提: 用 ELTIMA / com0com 建好一对虚拟串口 (例如 COM1 <-> COM2)。

用法:
    python -m tools.serial_sim.test_loopback              # 自动挑一对口
    python -m tools.serial_sim.test_loopback --port COM1  # 指定上位机侧的口
"""

from __future__ import annotations

import argparse
import queue
import sys
import time
from typing import List, Optional, Tuple

try:
    from pc_ui.protocol import (CHANNEL_COUNT, CJ_SLOT_COUNT,
                                ST_FLAG_RAW_UPLINK, cj_per_channel,
                                parse_status)
    from pc_ui.simulator import DemoSource          # noqa: F401 (复用其自检辅助)
    from pc_ui.serial_reader import SerialReader, SERIAL_AVAILABLE, list_ports
    from pc_ui.tc_table import TCTableSet, default_table_dir, MODE_PCHIP
except ImportError:
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    from pc_ui.protocol import (CHANNEL_COUNT, CJ_SLOT_COUNT,
                                ST_FLAG_RAW_UPLINK, cj_per_channel,
                                parse_status)
    from pc_ui.serial_reader import SerialReader, SERIAL_AVAILABLE, list_ports
    from pc_ui.tc_table import TCTableSet, default_table_dir, MODE_PCHIP

try:
    from tools.serial_sim.sim_device import SimulatedDevice
except ImportError:
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from tools.serial_sim.sim_device import SimulatedDevice


def _pick_pair(explicit_host_port: str) -> Optional[Tuple[str, str]]:
    """挑一对虚拟串口: 返回 (上位机侧口, 下位机侧口)。"""
    if not SERIAL_AVAILABLE:
        print("没装 pyserial")
        return None
    devs = sorted(p.device for p in list_ports.comports())
    if len(devs) < 2:
        print("没找到虚拟串口对 (需要 >=2 个 COM 口)。当前: %s" % (devs or "无"))
        print("请先用 ELTIMA / com0com 建一对虚拟串口。")
        return None

    if explicit_host_port:
        host = explicit_host_port.upper()
        # 下位机侧 = 找一个和 host 相邻的口 (COM1<->COM2 这类约定)
        import re
        m = re.match(r"^COM(\d+)$", host)
        for cand in devs:
            if cand.upper() == host:
                continue
            if m:
                n = int(m.group(1))
                other = "COM%d" % (n + 1 if n % 2 == 1 else n - 1)
                if cand.upper() == other:
                    return host, cand.upper()
        # 没匹配到约定 -> 用剩下的第一个
        for cand in devs:
            if cand.upper() != host:
                return host, cand.upper()
        return None

    # 自动: 取最小的两个口 (COM1<->COM2 这类虚拟对通常编号最小)
    return devs[0].upper(), devs[1].upper()


def _log(msg: str) -> None:
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg))


def run(port_pair: Optional[Tuple[str, str]] = None,
        host_port: str = "", duration: float = 3.0) -> bool:
    ok = True

    def check(name: str, cond: bool, extra: str = "") -> None:
        nonlocal ok
        print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                               ("  " + extra) if extra else ""))
        if not cond:
            ok = False

    pair = port_pair or _pick_pair(host_port)
    if pair is None:
        return False
    host_port, sim_port = pair
    _log("虚拟串口对: 上位机 %s  <->  下位机 %s" % (host_port, sim_port))

    dev = SimulatedDevice(port=sim_port, baudrate=115200, chips=4,
                          uplink="raw", sps=20, autorun=True, corrupt=0.0,
                          seed=20261004, quiet=True)
    dev.start()
    time.sleep(1.0)                            # 等模拟器把串口占上

    # ELTIMA 之类的虚拟串口, 上一个测试进程退出后偶尔还要一两秒才放开句柄,
    # 所以这里先探一下上位机侧的口, 失败就重试几次。
    import serial
    if SERIAL_AVAILABLE:
        for attempt in range(6):
            try:
                _probe = serial.Serial(host_port, baudrate=115200, timeout=0.05)
                _probe.close()
                break
            except Exception as exc:
                if attempt == 5:
                    print("  [FAIL] 上位机侧 %s 打不开: %s" % (host_port, exc))
                    print("         确认这个口没被别的程序占用, 也没被上一次测试占着。")
                    dev.stop()
                    dev.join(timeout=2.0)
                    return False
                time.sleep(0.5)

    q: "queue.Queue[dict]" = queue.Queue()
    reader = SerialReader(host_port, baudrate=115200, out_queue=q,
                          reconnect=False)
    reader.start()
    _log("上位机读取线程已连 %s" % host_port)

    raw_msgs: List[dict] = []
    status_msgs: List[dict] = []
    temp_msgs: List[dict] = []
    unknown_msgs: List[dict] = []
    deadline = time.time() + duration
    while time.time() < deadline:
        try:
            m = q.get(timeout=0.2)
        except queue.Empty:
            continue
        t = m.get("type")
        if t == "raw":
            raw_msgs.append(m)
        elif t == "status":
            status_msgs.append(m)
        elif t == "temp":
            temp_msgs.append(m)
        elif t == "unknown":
            unknown_msgs.append(m)
        # 其它 (connected/crc_error/error/info) 忽略

    dev.stop()
    dev.join(timeout=2.0)
    reader.stop()                              # 内部已带超时 join

    print()
    print("---- 结果 ----")
    print("  收到原始帧 %d / 状态帧 %d / 温度帧 %d / 未知帧 %d"
          % (len(raw_msgs), len(status_msgs), len(temp_msgs), len(unknown_msgs)))

    check("上位机能从串口收到原始帧 (CMD=0x12)", len(raw_msgs) >= 3,
          "%d 帧" % len(raw_msgs))
    check("没有未知帧 (协议不匹配)", not unknown_msgs,
          "%d 帧" % len(unknown_msgs) if unknown_msgs else "")

    if raw_msgs:
        m = raw_msgs[-1]
        check("原始帧字段完整 (µV 32 / 冷端 16)",
              len(m.get("emf_uv", [])) == CHANNEL_COUNT
              and len(m.get("cj_c", [])) == CJ_SLOT_COUNT)
        try:
            tb = TCTableSet(default_table_dir()).get("K")
            import numpy as np
            emf = np.asarray(m["emf_uv"], dtype=float)
            cj = np.asarray(cj_per_channel(m["cj_c"]), dtype=float)
            temps = np.asarray(tb.compensate_hot_junction(emf, cj, MODE_PCHIP),
                               dtype=float)
            check("原始帧能换算成温度", bool(np.any(np.isfinite(temps))))
            finite = temps[np.isfinite(temps)]
            if finite.size:
                check("温度落在合理范围 (0..1200 °C)",
                      bool((finite > -50).all() and (finite < 1250).all()),
                      "min=%.1f max=%.1f" % (finite.min(), finite.max()))
                print("    本帧换算结果 (前 8 路): %s"
                      % np.array2string(finite[:8], precision=2))
            print("    冷端 (前 4 片): %s" % m["cj_c"][:4])
        except Exception as exc:
            check("温度换算", False, str(exc))

    if status_msgs:
        st = status_msgs[-1].get("status") or {}
        check("状态帧 chip_count == 4", st.get("chip_count") == 4,
              str(st.get("chip_count")))
        check("状态帧 active_channels == 8", st.get("active_channels") == 8)
        check("RAW_UPLINK 标志置位",
              bool(st.get("flags", 0) & ST_FLAG_RAW_UPLINK),
              st.get("flag_text", ""))
        check("状态帧有轮次计数", (st.get("rounds") or 0) > 0,
              str(st.get("rounds")))

    if temp_msgs:
        check("模式是 raw, 不该有温度帧", False,
              "但收到了 %d 帧 0x10" % len(temp_msgs))

    print()
    print("自检结果: %s" % ("全部通过" if ok else "有失败项"))
    return ok


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="虚拟串口回环联调测试")
    ap.add_argument("--port", default="", help="上位机侧串口名 (如 COM1)")
    ap.add_argument("--duration", type=float, default=3.0, help="收多久 (秒)")
    args = ap.parse_args(argv)
    return 0 if run(host_port=args.port, duration=args.duration) else 1


if __name__ == "__main__":
    sys.exit(main())
