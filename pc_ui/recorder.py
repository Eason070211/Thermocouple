#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据记录模块 —— 后台线程 + Pandas 追加写 CSV。

设计要点
--------
* UI 线程只调用 :meth:`DataRecorder.submit` (往队列里放一行, 立即返回),
  真正的磁盘 I/O 在记录线程里做, 所以 **点击"开始记录"不会让界面卡住**。
* 攒够 ``flush_rows`` 行或超过 ``flush_interval`` 秒就批量落盘一次,
  既保证掉电最多丢 1 秒数据, 又不会每秒写上千次文件。
* 队列满了就丢最旧的行并计数, 绝不阻塞采集 (丢数据比卡界面好)。
* 温度文件: ``temperature_YYYYMMDD_HHMMSS.csv``
  状态文件: ``status_YYYYMMDD_HHMMSS.csv`` (同一时间戳后缀, 便于配对)

温度 CSV 列:
    iso_time, unix_ts, elapsed_s, valid_count, ch00 .. ch31     (无效 = NaN)
状态 CSV 列:
    iso_time, unix_ts, elapsed_s, run, dr_sps, reject, chip_ok, chip_err,
    rounds, uptime_ms, drdy_cnt, spi_err, open_cnt, timeout_cnt, flags, flag_text
"""

from __future__ import annotations

import os
import queue
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Sequence

import numpy as np
import pandas as pd

try:                                  # 允许 "python pc_ui/main.py" 直接跑
    from .protocol import CHANNEL_COUNT, CHIP_COUNT, CJ_SLOT_COUNT
except ImportError:                   # pragma: no cover
    from protocol import CHANNEL_COUNT, CHIP_COUNT, CJ_SLOT_COUNT

#: 温度 CSV 的列。
#: ★ 除了算好的温度 (ch00..)，还同时存**原始热电势** (uv00..) 和**冷端温度** (cj00..)。
#:   这几个原始量是"换分度表后离线重算"的全部输入:
#:       hot = table.compensate_hot_junction(uv[ch], cj[ch // 2])
#:   所以哪怕当初用 K 型录的数据，事后换成 J 型表也能重新算一遍，
#:   不需要重新做实验。温度帧模式 (固件自己算) 下这两组列填 NaN。
TEMP_COLUMNS = ["iso_time", "unix_ts", "elapsed_s", "valid_count"] + \
               ["ch%02d" % i for i in range(CHANNEL_COUNT)] + \
               ["uv%02d" % i for i in range(CHANNEL_COUNT)] + \
               ["cj%02d" % i for i in range(CJ_SLOT_COUNT)]

STATUS_COLUMNS = ["iso_time", "unix_ts", "elapsed_s", "run", "dr_sps", "reject",
                  "chip_ok", "chip_err", "rounds", "uptime_ms", "drdy_cnt",
                  "spi_err", "open_cnt", "timeout_cnt", "flags", "flag_text"]

_SENTINEL_STOP = "__STOP__"


def _pad_to(seq: Optional[Sequence[float]], n: int) -> np.ndarray:
    """把可选序列补齐/截断到 n 个 float; ``None`` -> 全 NaN。"""
    if seq is None:
        return np.full(n, np.nan)
    a = np.asarray(list(seq), dtype=float)
    if a.size < n:
        a = np.concatenate([a, np.full(n - a.size, np.nan)])
    return a[:n]


def build_filename(prefix: str, when: Optional[datetime] = None,
                   directory: str = ".") -> str:
    """生成 ``<prefix>_YYYYMMDD_HHMMSS.csv`` 的完整路径。"""
    when = when or datetime.now()
    return os.path.join(directory, "%s_%s.csv" % (prefix, when.strftime("%Y%m%d_%H%M%S")))


class DataRecorder(threading.Thread):
    """把温度/状态数据流式写入 CSV 的后台线程。"""

    def __init__(self, flush_rows: int = 64, flush_interval: float = 1.0,
                 queue_size: int = 20000):
        """
        :param flush_rows:     攒够多少行强制落盘
        :param flush_interval: 最长多久落盘一次 (秒)
        :param queue_size:     行队列上限; 满了丢最旧的行并计入 dropped
        """
        super().__init__(name="DataRecorder", daemon=True)
        self.flush_rows = int(flush_rows)
        self.flush_interval = float(flush_interval)
        self._in_queue: "queue.Queue[tuple]" = queue.Queue(maxsize=queue_size)

        self._lock = threading.Lock()
        self._active = False
        self._rows_written = 0
        self._status_rows_written = 0
        self.dropped = 0

        self.directory: Optional[str] = None
        self.temp_path: Optional[str] = None
        self.status_path: Optional[str] = None
        self._t0: float = 0.0
        self._temp_header_written = False
        self._status_header_written = False
        self._last_error: Optional[str] = None

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------
    @property
    def active(self) -> bool:
        """是否正在记录 (UI 用它决定按钮文字/指示灯)。"""
        with self._lock:
            return self._active

    @property
    def rows_written(self) -> int:
        with self._lock:
            return self._rows_written

    @property
    def status_rows_written(self) -> int:
        with self._lock:
            return self._status_rows_written

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    # ------------------------------------------------------------------
    # 控制
    # ------------------------------------------------------------------
    def start_recording(self, directory: str = "./data", prefix: str = "temperature",
                        status_prefix: str = "status") -> str:
        """新建 CSV 并启动记录线程; 返回温度文件路径。"""
        if self.active:
            return self.temp_path or ""
        directory = os.path.abspath(os.path.expanduser(directory or "./data"))
        os.makedirs(directory, exist_ok=True)

        stamp = datetime.now()
        self.directory = directory
        self.temp_path = build_filename(prefix, stamp, directory)
        self.status_path = build_filename(status_prefix, stamp, directory)
        self._t0 = time.time()
        self._rows_written = 0
        self._status_rows_written = 0
        self.dropped = 0
        self._temp_header_written = False
        self._status_header_written = False
        self._last_error = None
        self._drain_queue()

        with self._lock:
            self._active = True
        if not self.is_alive():
            self.start()
        else:
            # 线程复用时用一个空标记唤醒它继续干活
            self._in_queue.put(("wake", None, None))
        return self.temp_path

    def stop_recording(self, timeout: float = 5.0) -> Dict[str, Optional[str]]:
        """停止记录, 落盘剩余数据并等待线程收尾。"""
        with self._lock:
            was_active = self._active
            self._active = False
        if was_active and self.is_alive():
            self._in_queue.put((_SENTINEL_STOP, None, None))
            self.join(timeout=timeout)
        return {"temp_path": self.temp_path, "status_path": self.status_path,
                "rows": self.rows_written, "status_rows": self.status_rows_written,
                "dropped": self.dropped, "error": self._last_error}

    # ------------------------------------------------------------------
    # 数据入口 (UI 线程调用, 非阻塞)
    # ------------------------------------------------------------------
    def submit(self, t: float, temps: Sequence[float],
               emf_uv: Optional[Sequence[float]] = None,
               cj_c: Optional[Sequence[float]] = None) -> None:
        """提交一行温度 (32 路, 无效通道填 NaN)。

        :param emf_uv: 可选的 32 路原始热电势 (µV, 来自 CMD=0x12)。
                       一起存下来, 以后换分度表可以**离线重算**历史数据。
        :param cj_c:   可选的 16 路冷端温度 (°C)。重算时同样必需 —— 冷端补偿
                       必须由同一个算法完成, 只存 µV 是算不出热端温度的。
        """
        if not self.active:
            return
        self._put(("temp", t, (
            list(temps)[:CHANNEL_COUNT],
            None if emf_uv is None else list(emf_uv)[:CHANNEL_COUNT],
            None if cj_c is None else list(cj_c)[:CJ_SLOT_COUNT],
        )))

    def submit_status(self, t: float, status: Dict) -> None:
        """提交一行状态帧。"""
        if not self.active:
            return
        self._put(("status", t, dict(status)))

    def _put(self, item: tuple) -> None:
        try:
            self._in_queue.put_nowait(item)
        except queue.Full:
            # 队列爆了: 丢最旧的一行, 保住实时性
            try:
                self._in_queue.get_nowait()
                self._in_queue.put_nowait(item)
            except queue.Empty:       # pragma: no cover
                pass
            self.dropped += 1

    def _drain_queue(self) -> None:
        while True:
            try:
                self._in_queue.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------------
    # 线程主体
    # ------------------------------------------------------------------
    def run(self) -> None:
        temp_rows: List[list] = []
        status_rows: List[list] = []
        last_flush = time.time()

        while True:
            timeout = max(0.01, self.flush_interval - (time.time() - last_flush))
            try:
                kind, t, payload = self._in_queue.get(timeout=timeout)
            except queue.Empty:
                kind = None
                t = payload = None

            if kind == _SENTINEL_STOP:
                break

            if kind == "temp":
                temp_rows.append(self._make_temp_row(t, payload))
            elif kind == "status":
                status_rows.append(self._make_status_row(t, payload))

            due = (time.time() - last_flush) >= self.flush_interval
            if temp_rows and (len(temp_rows) >= self.flush_rows or due):
                self._flush_temp(temp_rows)
                temp_rows = []
                last_flush = time.time()
            if status_rows and (len(status_rows) >= self.flush_rows or due):
                self._flush_status(status_rows)
                status_rows = []

            if not self.active and self._in_queue.empty():
                # 停止记录后把尾巴写完再退出
                self._flush_temp(temp_rows)
                self._flush_status(status_rows)
                temp_rows, status_rows = [], []
                break

        self._flush_temp(temp_rows)
        self._flush_status(status_rows)

    # ------------------------------------------------------------------
    # 行构造 / 落盘
    # ------------------------------------------------------------------
    def _make_temp_row(self, t: float, payload) -> list:
        """payload = (temps, emf_uv|None, cj_c|None) -> 一行 CSV。

        emf_uv / cj_c 为 None (固件发的是温度帧) 时, 对应列填 NaN,
        列集合保持不变 —— 这样同一个 CSV 里两种来源的行都能放。
        """
        temps, emf_uv, cj_c = payload
        values = np.asarray(list(temps), dtype=float)
        if values.size < CHANNEL_COUNT:
            values = np.concatenate([values, np.full(CHANNEL_COUNT - values.size, np.nan)])
        valid = int(np.count_nonzero(~np.isnan(values)))
        return [datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                round(float(t), 6), round(float(t) - self._t0, 4), valid] + \
               [float(v) for v in values] + \
               [float(v) for v in _pad_to(emf_uv, CHANNEL_COUNT)] + \
               [float(v) for v in _pad_to(cj_c, CJ_SLOT_COUNT)]

    def _make_status_row(self, t: float, status: Dict) -> list:
        return [datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                round(float(t), 6), round(float(t) - self._t0, 4),
                status.get("run"), status.get("dr_sps"), status.get("reject"),
                status.get("chip_ok"), status.get("chip_err"), status.get("rounds"),
                status.get("uptime_ms"), status.get("drdy_cnt"), status.get("spi_err"),
                status.get("open_cnt"), status.get("timeout_cnt"),
                status.get("flags"), status.get("flag_text")]

    def _append(self, path: Optional[str], columns: List[str], rows: List[list],
                header_flag: str) -> None:
        if not path or not rows:
            return
        try:
            frame = pd.DataFrame(rows, columns=columns)
            first = not getattr(self, header_flag)
            frame.to_csv(path, mode="a", header=first, index=False,
                         float_format="%.4f", na_rep="NaN")
            setattr(self, header_flag, True)
        except Exception as exc:      # 磁盘满/权限问题 -> 记下来, 不抛出
            self._last_error = str(exc)

    def _flush_temp(self, rows: List[list]) -> None:
        if not rows:
            return
        self._append(self.temp_path, TEMP_COLUMNS, rows, "_temp_header_written")
        with self._lock:
            self._rows_written += len(rows)
        rows.clear()

    def _flush_status(self, rows: List[list]) -> None:
        if not rows:
            return
        self._append(self.status_path, STATUS_COLUMNS, rows, "_status_header_written")
        with self._lock:
            self._status_rows_written += len(rows)
        rows.clear()
