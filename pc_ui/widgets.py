#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自定义 Tkinter 控件 —— 全部与业务逻辑解耦, 只负责"显示"。

包含:
    * :class:`RingBuffer`       —— numpy 环形缓冲 (最近 N 点, 供实时曲线使用)
    * :class:`StatusLamp`       —— 状态指示灯 (连接 / 记录)
    * :class:`ChannelGrid`      —— 32 路实时数值网格 (通道号 / 温度 / 状态)
    * :class:`ChannelSelector`  —— 32 路曲线显示/隐藏勾选框
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:                                  # 允许 "python pc_ui/main.py" 直接跑
    from .protocol import CHANNEL_COUNT
except ImportError:                   # pragma: no cover
    from protocol import CHANNEL_COUNT

# ---------------------------------------------------------------------------
# 状态文本 / 颜色 (需求: 正常 / 断线 / 超温)
# ---------------------------------------------------------------------------
STATE_NORMAL = "正常"
STATE_OPEN = "断线"
STATE_OVER = "超温"

STATE_COLORS: Dict[str, str] = {
    STATE_NORMAL: "#1a7f37",          # 绿
    STATE_OPEN: "#c62828",            # 红
    STATE_OVER: "#ef6c00",            # 橙
}
LAMP_COLORS = {
    "off": "#9e9e9e",
    "ok": "#22c55e",
    "warn": "#f59e0b",
    "error": "#ef4444",
    "rec": "#ef4444",
    "rec_dim": "#7f1d1d",
    "busy": "#3b82f6",
}


# ---------------------------------------------------------------------------
# 环形缓冲
# ---------------------------------------------------------------------------
class RingBuffer:
    """预分配的 numpy 环形缓冲: 固定内存, 只保留最近 ``capacity`` 个采样点。

    形状: ``t`` 为 (capacity,), ``v`` 为 (capacity, channels)。
    ``snapshot()`` 返回按时间从旧到新排好序的副本 (供 matplotlib 直接使用)。
    """

    def __init__(self, capacity: int = 1000, channels: int = CHANNEL_COUNT):
        self.channels = channels
        self.capacity = max(2, int(capacity))
        self.clear()

    def clear(self) -> None:
        self._t = np.full(self.capacity, np.nan, dtype=np.float64)
        self._v = np.full((self.capacity, self.channels), np.nan, dtype=np.float32)
        self._head = 0          # 满环时指向最旧的一个点
        self._count = 0

    def resize(self, capacity: int) -> None:
        """改变保留点数; 已有数据按"保留最新"的原则迁移。"""
        capacity = max(2, int(capacity))
        if capacity == self.capacity:
            return
        if self._count:
            t, v = self.snapshot()
            keep = min(capacity, len(t))
            t, v = t[-keep:], v[-keep:]
        else:
            t = v = None
        self.capacity = capacity
        self.clear()
        if t is not None:
            n = len(t)
            self._t[:n] = t
            self._v[:n] = v
            self._count = n
            self._head = n % capacity if n == capacity else 0

    @property
    def count(self) -> int:
        return self._count

    @property
    def last_time(self) -> Optional[float]:
        if not self._count:
            return None
        return float(self._t[(self._head - 1) % self.capacity])

    def push(self, t: float, values: Sequence[float]) -> None:
        values = np.asarray(values, dtype=np.float32).reshape(-1)
        if values.size < self.channels:
            values = np.concatenate([values, np.full(self.channels - values.size, np.nan)])
        if self._count < self.capacity:
            index = self._count
            self._count += 1
        else:
            index = self._head
            self._head = (self._head + 1) % self.capacity
        self._t[index] = t
        self._v[index] = values[:self.channels]

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (t, v), 均为从旧到新的连续副本; 空缓冲返回零长度数组。"""
        if self._count == 0:
            return (np.empty(0, dtype=np.float64),
                    np.empty((0, self.channels), dtype=np.float32))
        if self._count < self.capacity:
            return self._t[:self._count].copy(), self._v[:self._count].copy()
        h = self._head
        return (np.concatenate([self._t[h:], self._t[:h]]),
                np.concatenate([self._v[h:], self._v[:h]]))


# ---------------------------------------------------------------------------
# 状态指示灯
# ---------------------------------------------------------------------------
def theme_background(widget: tk.Misc, fallback: str = "#f0f0f0") -> str:
    """取 ttk 主题的窗口底色, 让 tk.Canvas/Label 和周围 ttk 控件颜色一致。"""
    try:
        return ttk.Style(widget).lookup("TFrame", "background") or fallback
    except Exception:                 # pragma: no cover
        return fallback


class StatusLamp(ttk.Frame):
    """一个小圆灯 + 文字, 用于"连接"和"记录"状态。"""

    def __init__(self, master, text: str = "", size: int = 14, **kw):
        super().__init__(master, **kw)
        self._canvas = tk.Canvas(self, width=size, height=size, highlightthickness=0,
                                 bd=0, bg=theme_background(master))
        self._canvas.grid(row=0, column=0, padx=(0, 4))
        half = size // 2
        self._dot = self._canvas.create_oval(1, 1, size - 1, size - 1,
                                             fill=LAMP_COLORS["off"], outline="#666666")
        self._label = ttk.Label(self, text=text, width=10)
        self._label.grid(row=0, column=1, sticky="w")
        self.set_state("off", text)

    def set_state(self, state: str, text: Optional[str] = None) -> None:
        """state: off / ok / warn / error / rec / rec_dim / busy"""
        self._canvas.itemconfigure(self._dot, fill=LAMP_COLORS.get(state, LAMP_COLORS["off"]))
        if text is not None:
            self._label.configure(text=text)


# ---------------------------------------------------------------------------
# 32 路数值网格
# ---------------------------------------------------------------------------
class ChannelGrid(ttk.Frame):
    """32 路当前温度 + 状态, 网格布局 (默认 2 列 x 16 行)。

    为了手感流畅, 只在文本真正变化时才调用 ``configure``, 避免每 100ms 无谓重绘。
    """

    def __init__(self, master, channels: int = CHANNEL_COUNT, columns: int = 2,
                 over_temp: float = 1000.0, **kw):
        super().__init__(master, **kw)
        self.channels = channels
        self.columns = columns
        self.over_temp = float(over_temp)
        #: 当前"真实存在"的通道数 (由固件状态帧的片数决定); 其余格子隐藏不显示
        self.active = channels
        self._temp_labels: List[tk.Label] = []
        self._state_labels: List[tk.Label] = []
        self._sigma_labels: List[tk.Label] = []
        self._cells: List[tk.Frame] = []
        self._cache: List[Tuple[str, str]] = [("", "")] * channels
        self._sigma_cache: List[str] = [""] * channels

        for ch in range(channels):
            row, col = divmod(ch, columns)
            cell = tk.Frame(self, relief="groove", bd=1, padx=4, pady=1)
            cell.grid(row=row, column=col, sticky="nsew", padx=2, pady=1)
            self.rowconfigure(row, weight=1)
            self.columnconfigure(col, weight=1)

            chip, half = ch // 2, "A" if ch % 2 == 0 else "B"
            ttk.Label(cell, text="CH%02d  #%02d%s" % (ch, chip, half),
                      font=("Consolas", 8)).grid(row=0, column=0, sticky="w")
            temp = tk.Label(cell, text="---.--", font=("Consolas", 12, "bold"),
                            width=7, anchor="e")
            temp.grid(row=1, column=0, sticky="w")
            ttk.Label(cell, text="°C", font=("Segoe UI", 8)).grid(row=1, column=1, sticky="w")
            state = tk.Label(cell, text=STATE_NORMAL, font=("Microsoft YaHei", 8),
                             width=4, anchor="e")
            state.grid(row=1, column=2, sticky="e")
            cell.columnconfigure(2, weight=1)
            # ★ 噪声 σ: 最近若干帧的标准差, 用来一眼看出哪一路抖得厉害
            sigma = tk.Label(cell, text="σ ---", font=("Consolas", 7),
                             fg="#888888", anchor="w")
            sigma.grid(row=2, column=0, columnspan=3, sticky="w")

            self._cells.append(cell)
            self._temp_labels.append(temp)
            self._state_labels.append(state)
            self._sigma_labels.append(sigma)

    # ------------------------------------------------------------------
    def set_active_channels(self, count: int) -> None:
        """只显示前 ``count`` 路 (本板实际通道数), 其余格子直接隐藏。

        ★ 8 路板就只显示 8 格, 不会因为协议里那 24 个恒为 NaN 的槽位
          而满屏"断线", 也不会让人误以为板子坏了。
        值没变时直接返回, 所以可以在每个状态帧里放心调用。
        """
        count = max(1, min(int(count), self.channels))
        if count == self.active:
            return
        self.active = count
        for ch, cell in enumerate(self._cells):
            if ch < count:
                cell.grid()                      # 恢复原来记住的 grid 位置
            else:
                cell.grid_remove()
                self._cache[ch] = ("", "")       # 清缓存, 以后重新出现时能刷新

    def set_over_temp(self, value: float) -> None:
        self.over_temp = float(value)

    def classify(self, value: float) -> str:
        """温度 -> 状态文本 (断线 / 超温 / 正常)。"""
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return STATE_OPEN
        if value > self.over_temp:
            return STATE_OVER
        return STATE_NORMAL

    def update_values(self, values: Sequence[float],
                      sigmas: Optional[Sequence[float]] = None) -> None:
        """刷新 32 路显示; ``values`` 里 NaN 表示断线。

        :param sigmas: 可选的每通道噪声标准差 (σ, °C)。给了就在每格下面显示
                       "σ x.xx", 抖动大的通道标红; None 时显示 "σ ---"。
        """
        for ch in range(self.channels):
            value = float(values[ch]) if ch < len(values) else float("nan")
            state = self.classify(value)
            temp_text = "---.--" if math.isnan(value) else "%7.2f" % value
            if (temp_text, state) != self._cache[ch]:
                self._cache[ch] = (temp_text, state)
                color = STATE_COLORS[state]
                self._temp_labels[ch].configure(text=temp_text, fg=color)
                self._state_labels[ch].configure(text=state, fg=color)

            # ★ 噪声 σ (只在文本变化时 configure, 避免每 100ms 无谓重绘)
            sigma = None
            if sigmas is not None and ch < len(sigmas):
                s = float(sigmas[ch])
                sigma = s if math.isfinite(s) else None
            sigma_text = "σ ---" if sigma is None else "σ %.2f" % sigma
            if sigma_text != self._sigma_cache[ch]:
                self._sigma_cache[ch] = sigma_text
                noisy = sigma is not None and sigma > 1.0
                self._sigma_labels[ch].configure(
                    text=sigma_text, fg="#c62828" if noisy else "#888888")

    def reset(self) -> None:
        self._sigma_cache = [""] * self.channels
        self.update_values([float("nan")] * self.channels)


# ---------------------------------------------------------------------------
# 32 路曲线勾选框
# ---------------------------------------------------------------------------
class ChannelSelector(ttk.LabelFrame):
    """32 个勾选框 (带曲线颜色), 控制哪几路画在图上。"""

    def __init__(self, master, colors: Sequence, columns: int = 8,
                 on_change: Optional[Callable[[np.ndarray], None]] = None, **kw):
        super().__init__(master, text="曲线显示通道", **kw)
        self.colors = list(colors)
        self.on_change = on_change
        self.channels = len(self.colors)
        self.vars = [tk.BooleanVar(value=True) for _ in range(self.channels)]
        #: 当前"真实存在"的通道数; 其余勾选框禁用并取消勾选 (曲线也不画)
        self.active = self.channels

        # 用 tk.Checkbutton 而不是 ttk 的: ttk 样式无法逐个控件改前景色,
        # 而这里希望勾选框文字颜色 = 该通道曲线的颜色, 便于对照。
        body = ttk.Frame(self)
        body.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        bg = theme_background(body)
        self.boxes: List[tk.Checkbutton] = []
        for ch in range(self.channels):
            row, col = divmod(ch, columns)
            box = tk.Checkbutton(body, text="CH%02d" % ch, variable=self.vars[ch],
                                 command=self._changed, fg=self.colors[ch],
                                 bg=bg, activebackground=bg,
                                 activeforeground=self.colors[ch], selectcolor="#ffffff",
                                 bd=0, highlightthickness=0, anchor="w", padx=0)
            box.grid(row=row, column=col, sticky="w", padx=1)
            self.boxes.append(box)

        buttons = ttk.Frame(self)
        buttons.grid(row=1, column=0, sticky="ew", padx=2, pady=(0, 2))
        ttk.Button(buttons, text="全选", width=6,
                   command=lambda: self._set_all(True)).grid(row=0, column=0, padx=1)
        ttk.Button(buttons, text="全不选", width=6,
                   command=lambda: self._set_all(False)).grid(row=0, column=1, padx=1)
        ttk.Button(buttons, text="反选", width=6,
                   command=self._invert).grid(row=0, column=2, padx=1)
        self.visible_label = ttk.Label(buttons, text="")
        self.visible_label.grid(row=0, column=3, padx=6)
        self._update_label()

    # ------------------------------------------------------------------
    def set_active_channels(self, count: int) -> None:
        """只让前 ``count`` 路可勾选 (本板实际通道数)。

        其余勾选框置灰并取消勾选 —— 这样 visible_mask() 天然不会选中不存在的通道,
        曲线层也不需要额外判断。值没变时直接返回。
        """
        count = max(1, min(int(count), self.channels))
        if count == self.active:
            return
        self.active = count
        for ch, box in enumerate(self.boxes):
            if ch < count:
                box.configure(state="normal")
            else:
                box.configure(state="disabled")
                self.vars[ch].set(False)
        self._changed()

    def visible_mask(self) -> np.ndarray:
        return np.array([bool(v.get()) for v in self.vars], dtype=bool)

    def _changed(self) -> None:
        self._update_label()
        if self.on_change:
            self.on_change(self.visible_mask())

    def _set_all(self, value: bool) -> None:
        # "全选" 只作用于本板真实存在的通道
        for index, var in enumerate(self.vars):
            var.set(value and index < self.active)
        self._changed()

    def _invert(self) -> None:
        for index, var in enumerate(self.vars):
            var.set((not var.get()) if index < self.active else False)
        self._changed()

    def _update_label(self) -> None:
        count = int(self.visible_mask().sum())
        try:
            self.visible_label.configure(text="显示 %d/%d 路" % (count, self.active))
        except tk.TclError:           # pragma: no cover
            pass
