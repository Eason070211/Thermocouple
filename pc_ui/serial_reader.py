#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口读取线程 —— 唯一的串口 I/O 所有者。

设计要点
--------
* 读、写都在本线程内完成, **绝不在 UI 线程里碰串口**, 保证界面不阻塞。
* 解析好的数据通过 ``queue.Queue`` 交给 UI 线程 (线程安全, 无需加锁)。
* 断线自动重连: USB 掉线 / 串口异常时关闭句柄, 按固定间隔重试打开。
* CRC 失败由 :class:`~pc_ui.protocol.FrameParser` 直接丢弃, 这里只上报计数。

投递到队列的消息都是 dict, ``type`` 字段取值:

    {"type": "connected",    "port": ..., "baudrate": ...}
    {"type": "disconnected", "reason": ...}          # 物理断开
    {"type": "reconnecting", "attempt": n, "reason": ...}
    {"type": "temp",   "t": float, "temps": [float x32]}
    {"type": "status", "t": float, "status": {...}}
    {"type": "unknown","cmd": int, "length": int, "t": float}
    {"type": "crc_error", "count": int}              # 累计 CRC 错帧数
    {"type": "error",  "message": str}
    {"type": "info",   "message": str}
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Dict, List, Optional

try:                                  # 允许 "python pc_ui/main.py" 直接跑
    from .protocol import (FRAME_GAP_S, HEAD_UP, CMD_UP_STATUS, CMD_UP_TEMP,
                           FrameParser, build_frame, parse_status, parse_temp)
except ImportError:                   # pragma: no cover
    from protocol import (FRAME_GAP_S, HEAD_UP, CMD_UP_STATUS, CMD_UP_TEMP,
                          FrameParser, build_frame, parse_status, parse_temp)

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except ImportError:                   # pragma: no cover - 没装 pyserial 时 UI 仍可启动
    serial = None
    list_ports = None
    SERIAL_AVAILABLE = False


def available_ports() -> List[str]:
    """扫描当前可用的串口名列表 (按名称排序)。"""
    if not SERIAL_AVAILABLE:
        return []
    try:
        return sorted(p.device for p in list_ports.comports())
    except Exception:                 # pragma: no cover - 驱动异常时别让 UI 崩
        return []


def describe_ports() -> Dict[str, str]:
    """串口名 -> 描述文本 (下拉框里显示得更友好)。"""
    if not SERIAL_AVAILABLE:
        return {}
    result: Dict[str, str] = {}
    try:
        for port in list_ports.comports():
            desc = port.description or ""
            result[port.device] = ("%s - %s" % (port.device, desc)) if desc else port.device
    except Exception:                 # pragma: no cover
        pass
    return result


class SerialReader(threading.Thread):
    """后台线程: 独占串口, 收帧进队列, 发帧走发送队列。"""

    def __init__(self,
                 port: str,
                 baudrate: int = 115200,
                 out_queue: Optional["queue.Queue[Dict]"] = None,
                 reconnect: bool = True,
                 reconnect_interval: float = 1.0,
                 max_reconnect: int = 0,
                 read_timeout: float = 0.05,
                 read_size: int = 512,
                 open_settle: float = 0.10):
        """
        :param port:               串口名, 如 "COM3"
        :param baudrate:           115200 / 460800
        :param out_queue:          消息队列; None 时自建 (可通过 .out_queue 取用)
        :param reconnect:          掉线后是否自动重连
        :param reconnect_interval: 重连间隔 (秒)
        :param max_reconnect:      最大重连次数, 0 = 无限重试
        :param read_timeout:       串口读超时 (秒), 决定线程循环节拍
        :param read_size:          单次读取上限字节数 (一次最多约 3 帧温度)
        :param open_settle:        打开串口后的稳定等待 (秒); STM32 复位期间
                                   会输出噪声, 等它稳定再清空输入缓冲。
        """
        super().__init__(name="SerialReader-%s" % port, daemon=True)
        self.port = port
        self.baudrate = int(baudrate)
        self.out_queue: "queue.Queue[Dict]" = out_queue or queue.Queue()
        self.reconnect = reconnect
        self.reconnect_interval = reconnect_interval
        self.max_reconnect = max_reconnect
        self.read_timeout = read_timeout
        self.read_size = read_size
        self.open_settle = open_settle

        self._stop_event = threading.Event()
        self._tx_queue: "queue.Queue[bytes]" = queue.Queue()
        self._ser = None
        self._parser = FrameParser(HEAD_UP)
        self._last_byte_time = 0.0
        self._reported_crc = 0

    # ------------------------------------------------------------------
    # 对外 API (全部线程安全, UI 线程可直接调用)
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def send(self, frame: bytes) -> None:
        """把已组好的下行帧排入发送队列 (实际写在读取线程里完成)。"""
        if not self._stop_event.is_set():
            self._tx_queue.put(bytes(frame))

    def send_command(self, cmd: int, data: bytes = b"") -> None:
        """组帧并发送 (下行帧头 0xAA)。"""
        self.send(build_frame(cmd, data))

    def stop(self, timeout: float = 2.0) -> None:
        """请求线程退出并等待收尾。"""
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=timeout)

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------
    def _emit(self, **message) -> None:
        message.setdefault("t", time.time())
        self.out_queue.put(message)

    def _emit_stats(self) -> None:
        """CRC 错帧计数变化时上报一次。"""
        count = self._parser.stats["crc_errors"]
        if count != self._reported_crc:
            self._reported_crc = count
            self._emit(type="crc_error", count=count)

    def _try_open(self) -> bool:
        if not SERIAL_AVAILABLE:
            self._emit(type="error", message="未安装 pyserial: pip install pyserial")
            return False
        try:
            self._ser = serial.Serial(self.port, self.baudrate,
                                      timeout=self.read_timeout,
                                      write_timeout=1.0)
            # STM32 打开串口(DTR 跳变)时可能复位, 先让它安静下来再清缓冲
            time.sleep(self.open_settle)
            try:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            except Exception:
                pass
            self._parser.reset()
            self._last_byte_time = 0.0
            self._emit(type="connected", port=self.port, baudrate=self.baudrate)
            return True
        except Exception as exc:
            self._ser = None
            self._emit(type="error", message="打开 %s 失败: %s" % (self.port, exc))
            return False

    def _close_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _flush_tx(self) -> None:
        """把发送队列里的帧写出去 (最多 8 帧/轮, 避免饿死接收)。"""
        for _ in range(8):
            try:
                frame = self._tx_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._ser.write(frame)
            except Exception as exc:
                self._emit(type="error", message="发送失败: %s" % exc)
                self._handle_link_loss("发送失败: %s" % exc)
                return

    def _handle_link_loss(self, reason: str) -> None:
        self._close_serial()
        self._emit(type="disconnected", reason=reason)

    def _handle_frames(self, chunk: bytes) -> None:
        now = time.time()
        # 帧内字节间超时 -> 丢弃半截帧重新同步 (与固件的 UART_FRAME_GAP_MS 对应)
        if self._last_byte_time and (now - self._last_byte_time) > FRAME_GAP_S:
            self._parser.reset()
        self._last_byte_time = now

        for cmd, payload in self._parser.feed(chunk):
            if cmd == CMD_UP_TEMP:
                self._emit(type="temp", t=now, temps=parse_temp(payload),
                           length=len(payload))
            elif cmd == CMD_UP_STATUS:
                status = parse_status(payload)
                status["t"] = now
                self._emit(type="status", t=now, status=status)
            else:
                self._emit(type="unknown", cmd=cmd, length=len(payload), t=now)
        self._emit_stats()

    # ------------------------------------------------------------------
    # 线程主体
    # ------------------------------------------------------------------
    def run(self) -> None:
        attempts = 0
        while not self._stop_event.is_set():
            # 1) 确保串口是打开的
            if self._ser is None:
                if not self._try_open():
                    attempts += 1
                    if self.max_reconnect and attempts > self.max_reconnect:
                        self._emit(type="error", message="重连次数超限, 停止重连")
                        return
                    if not self.reconnect:
                        return
                    self._emit(type="reconnecting", attempt=attempts,
                               reason="串口未打开")
                    self._stop_event.wait(self.reconnect_interval)
                    continue
                attempts = 0

            # 2) 发送待发帧
            self._flush_tx()
            if self._stop_event.is_set():
                break

            # 3) 读数据 (read_timeout 决定阻塞上限, 因此循环不会卡死)
            try:
                pending = self._ser.in_waiting
            except Exception:
                pending = 0
            try:
                chunk = self._ser.read(min(max(pending, 1), self.read_size))
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                self._handle_link_loss("读取异常: %s" % exc)
                continue

            if chunk:
                self._handle_frames(chunk)
            elif self._ser is not None and not getattr(self._ser, "is_open", True):
                self._handle_link_loss("串口已关闭")

        # 退出清理
        self._flush_tx()
        self._close_serial()
        self._emit(type="info", message="串口线程已退出")
