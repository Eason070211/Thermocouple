#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TemperatureApp —— Tkinter 主界面 (32 路热电偶测温上位机)

界面布局
--------
    +----------------+---------------------------+------------------+
    |  左侧控制面板  |      中部实时曲线         |   右侧数值面板   |
    |  串口/采样率   |   Matplotlib 32 路曲线    |  32 路温度网格   |
    |  记录/绘图参数 |   通道勾选 / 工具栏       |  下位机状态详情  |
    +----------------+---------------------------+------------------+
    |                      底部状态栏 (消息/计数)                     |
    +--------------------------------------------------------------+

线程模型
--------
    * 串口线程 (SerialReader)  -> rx_queue -> **UI 线程** (root.after 100ms 轮询)
    * 记录线程 (DataRecorder)  <- submit() <- UI 线程 (只入队, 不落盘)
    * UI 线程只做: 取队列消息 / 更新曲线 / 更新数字 / 发命令 (命令也是入队)

    因此无论串口多快、磁盘多慢, 界面都不会被阻塞。
"""

from __future__ import annotations

import collections
import math
import os
import queue
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,  # noqa: E402
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure                              # noqa: E402

# 中文字体: Windows 用雅黑 / Linux 用 Noto, 避免中文标签变方框
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei",
                                          "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False                 # 负号正常显示
matplotlib.rcParams["path.simplify"] = True                       # 32 路曲线降采样加速
matplotlib.rcParams["path.simplify_threshold"] = 0.6

try:                                  # 允许 "python pc_ui/main.py" 直接跑
    from .protocol import (CHANNEL_COUNT, REJECT_TABLE, SAMPLE_RATE_CHOICES,
                           ST_FLAG_RAW_UPLINK, baudrate_advice, cj_per_channel,
                           cmd_run, cmd_set_rate_sps, cmd_single)
    from .recorder import DataRecorder
    from .serial_reader import SERIAL_AVAILABLE, SerialReader, available_ports
    from .logger import FileLogger
    from .simulator import DemoSource
    from .tc_table import DEFAULT_TYPE, MODE_PCHIP, TCTableSet, default_table_dir
    from .widgets import (ChannelGrid, ChannelSelector, RingBuffer, StatusLamp)
except ImportError:                   # pragma: no cover
    from protocol import (CHANNEL_COUNT, REJECT_TABLE, SAMPLE_RATE_CHOICES,
                          ST_FLAG_RAW_UPLINK, baudrate_advice, cj_per_channel,
                          cmd_run, cmd_set_rate_sps, cmd_single)
    from recorder import DataRecorder
    from serial_reader import SERIAL_AVAILABLE, SerialReader, available_ports
    from logger import FileLogger
    from simulator import DemoSource
    from tc_table import DEFAULT_TYPE, MODE_PCHIP, TCTableSet, default_table_dir
    from widgets import (ChannelGrid, ChannelSelector, RingBuffer, StatusLamp)

APP_TITLE = "热电偶温度监测上位机  ·  STM32F103 + N×ADS1220"


class TemperatureApp:
    """上位机主窗口。"""

    #: UI 刷新周期 (ms) —— 需求建议 100ms
    POLL_MS = 100
    #: 单次轮询最多处理多少条队列消息 (防止一次卡太久)
    MAX_MSGS_PER_TICK = 600
    #: 多久收不到温度帧就报警 (秒)
    DATA_TIMEOUT_S = 3.0

    def __init__(self,
                 demo: bool = False,
                 port: Optional[str] = None,
                 baudrate: int = 115200,
                 retention: int = 1000,
                 record_dir: str = "./data",
                 over_temp: float = 1000.0,
                 auto_connect: bool = True,
                 channels: int = 0,
                 tc_type: str = DEFAULT_TYPE):
        """
        :param channels: 强制指定本板通道数 (4/8/16/32); 0 = 自动,
                         由固件状态帧上报的片数决定 (推荐)。
        :param tc_type:  分度号 (默认 K)。必须是工程 tables/ 目录下已有表的类型。
        """
        self.demo = demo
        self.initial_port = port or ""
        self.initial_baud = int(baudrate)
        self.initial_retention = int(retention)
        self.initial_record_dir = record_dir
        self.initial_over_temp = float(over_temp)
        #: 用户强制的通道数 (0 = 跟随固件自动识别)
        self.forced_channels = int(channels or 0)
        #: 启动时选用的分度号 (默认 K)
        self.initial_tc_type = (tc_type or DEFAULT_TYPE).upper()
        #: 分度表集合 (扫描 tables/); 默认 K 型
        self.tc_tables = TCTableSet(default_table_dir())
        self.tc_table = None                     # 当前已加载的表

        # ---------------- 运行时状态 ----------------
        self.rx_queue: "queue.Queue[Dict]" = queue.Queue()
        self.reader = None                       # SerialReader 或 DemoSource
        self.recorder = DataRecorder()
        self.ring = RingBuffer(retention, CHANNEL_COUNT)
        self.latest = np.full(CHANNEL_COUNT, np.nan, dtype=float)
        self.visible = np.ones(CHANNEL_COUNT, dtype=bool)
        #: 本板真实存在的通道数。初值与控件保持一致(都是 32),
        #: 之后由状态帧或 --channels 参数收窄成实际值。
        self.active_channels = CHANNEL_COUNT
        self.status: Optional[Dict] = None
        self.over_temp = float(over_temp)
        self.t0: Optional[float] = None          # 会话第一帧的时间戳 (X 轴零点)
        self.paused = False
        self.temp_frames = 0
        self.crc_errors = 0
        self.unknown_frames = 0
        self.record_rows_base = 0
        self.last_temp_time = 0.0
        self.last_status_time = 0.0
        self.last_unknown_time = 0.0
        self._frame_times: "collections.deque[float]" = collections.deque(maxlen=400)
        self._single_pending = 0.0
        # ★ CMD=0x12 原始帧相关: 温度由上位机查分度表算, 这里记录换算统计
        self.raw_frames = 0                  # 收到的原始帧数
        self.raw_mode = False                # 固件是否处于"原始上报"模式 (状态帧标志)
        self._last_emf_uv = None             # 最近一帧的 32 路热电势 µV (给 CSV 用)
        self._raw_mode_logged = False        # "固件是原始模式"只提示一次
        self._raw_no_table_warned = False    # "表没加载"只提示一次
        self._raw_conv_warned = False        # "换算失败"只提示一次
        self._plot_dirty = True
        self._values_dirty = True
        self._closing = False
        self._timeout_warned = False
        self.log_lines: "collections.deque[str]" = collections.deque(maxlen=800)
        self.log_window: Optional[tk.Toplevel] = None
        # 文件日志: logs/ 目录 + 时间戳文件名, 每条带完整时间戳
        self.file_logger = FileLogger()

        # ---------------- 窗口 (必须先建 root, 再建 Tk 变量) ----------------
        self.root = tk.Tk()
        self.root.title(APP_TITLE + ("   [演示模式 DEMO]" if demo else ""))
        # 自适应屏幕: 小屏/高 DPI 下自动缩小, 保证右侧数值面板完整可见
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        width = max(1100, min(1600, screen_w - 24))
        height = max(660, min(960, screen_h - 70))
        self.root.geometry("%dx%d+%d+%d" % (width, height,
                                            max(0, (screen_w - width) // 2),
                                            max(0, (screen_h - height) // 3)))
        self.root.minsize(1040, 640)
        self._build_style()

        # ---------------- 变量 ----------------
        self.port_var = tk.StringVar(value=self.initial_port)
        self.baud_var = tk.StringVar(value=str(self.initial_baud))
        self.rate_var = tk.StringVar(value="%d SPS" % SAMPLE_RATE_CHOICES[0])
        self.reject_var = tk.StringVar(value=REJECT_TABLE[0])
        self.dir_var = tk.StringVar(value=os.path.abspath(self.initial_record_dir))
        self.retention_var = tk.IntVar(value=self.initial_retention)
        self.over_temp_var = tk.DoubleVar(value=self.initial_over_temp)
        self.autoscale_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="就绪")
        self.counters_var = tk.StringVar(value="")
        self.tc_type_var = tk.StringVar(value=self.initial_tc_type)
        self.tc_status_var = tk.StringVar(value="未加载")
        self.stat_vars: Dict[str, tk.StringVar] = {}

        # ---------------- 界面 ----------------
        self._build_layout()
        self._build_menu()
        self._bind_shortcuts()

        # 命令行强制指定了通道数就先按它显示 (不等状态帧)
        if self.forced_channels:
            self._apply_active_channels(self.forced_channels)

        # 加载默认分度表 (K 型)
        self.reload_tc_table()

        self.refresh_ports()
        self._log("界面已启动%s" % (" (演示模式: 使用虚拟下位机)" if demo else ""))
        if not SERIAL_AVAILABLE and not demo:
            self._log("未检测到 pyserial, 请执行: pip install pyserial", "error")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(self.POLL_MS, self._poll)

        if auto_connect and (demo or self.initial_port):
            self.root.after(300, lambda: self.connect(self.initial_port or None,
                                                      self.initial_baud))

    # ==================================================================
    # 界面搭建
    # ==================================================================
    def _build_style(self) -> None:
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():          # Windows 原生观感
            style.theme_use("vista")
        style.configure("TC.TCheckbutton", padding=1)
        style.configure("Section.TLabelframe.Label", font=("Microsoft YaHei", 9, "bold"))
        style.configure("Stat.TLabel", font=("Consolas", 9))
        style.configure("Big.TButton", font=("Microsoft YaHei", 9, "bold"), padding=4)

    def _build_layout(self) -> None:
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        main = ttk.Frame(self.root, padding=6)
        main.grid(row=0, column=0, sticky="nsew")
        main.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        self.left = ttk.Frame(main)
        self.left.grid(row=0, column=0, sticky="ns", padx=(0, 6))
        self.center = ttk.Frame(main)
        self.center.grid(row=0, column=1, sticky="nsew")
        self.center.rowconfigure(0, weight=1)
        self.center.columnconfigure(0, weight=1)
        self.right = ttk.Frame(main)
        self.right.grid(row=0, column=2, sticky="ns", padx=(6, 0))

        self._build_left_panel(self.left)
        self._build_plot(self.center)
        self._build_right_panel(self.right)
        self._build_statusbar()

    # ------------------------------------------------------------------
    def _build_left_panel(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)

        # ---------- 1. 串口连接 ----------
        box = ttk.LabelFrame(parent, text=" 串口连接 ", style="Section.TLabelframe")
        box.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="串口").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=14,
                                       state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(box, text="刷新", width=5,
                   command=self.refresh_ports).grid(row=0, column=2, padx=(2, 4), pady=2)

        ttk.Label(box, text="波特率").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        self.baud_combo = ttk.Combobox(box, textvariable=self.baud_var, width=14,
                                       state="readonly", values=["115200", "460800"])
        self.baud_combo.grid(row=1, column=1, columnspan=2, sticky="ew", padx=2, pady=2)

        ttk.Label(box, text="采样率").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        self.rate_combo = ttk.Combobox(box, textvariable=self.rate_var, width=14,
                                       state="readonly",
                                       values=["%d SPS" % s for s in SAMPLE_RATE_CHOICES])
        self.rate_combo.grid(row=2, column=1, columnspan=2, sticky="ew", padx=2, pady=2)
        self.rate_combo.bind("<<ComboboxSelected>>", lambda _e: self.apply_rate())

        ttk.Label(box, text="50/60Hz").grid(row=3, column=0, sticky="w", padx=4, pady=2)
        self.reject_combo = ttk.Combobox(box, textvariable=self.reject_var, width=14,
                                         state="readonly", values=list(REJECT_TABLE))
        self.reject_combo.grid(row=3, column=1, columnspan=2, sticky="ew", padx=2, pady=2)
        self.reject_combo.bind("<<ComboboxSelected>>", lambda _e: self.apply_rate())

        self.connect_btn = ttk.Button(box, text="连接", style="Big.TButton",
                                      command=self.on_connect_toggle)
        self.connect_btn.grid(row=4, column=0, columnspan=3, sticky="ew", padx=4, pady=(6, 4))

        lamps = ttk.Frame(box)
        lamps.grid(row=5, column=0, columnspan=3, sticky="w", padx=4, pady=(0, 4))
        self.lamp_conn = StatusLamp(lamps, text="未连接")
        self.lamp_conn.grid(row=0, column=0, sticky="w")
        self.lamp_rec = StatusLamp(lamps, text="未记录")
        self.lamp_rec.grid(row=0, column=1, sticky="w", padx=(8, 0))

        # ---------- 2. 采集控制 ----------
        box = ttk.LabelFrame(parent, text=" 采集控制 ", style="Section.TLabelframe")
        box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        box.columnconfigure(0, weight=1)
        box.columnconfigure(1, weight=1)
        ttk.Button(box, text="启动采集", command=lambda: self.acquire(True)
                   ).grid(row=0, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(box, text="停止采集", command=lambda: self.acquire(False)
                   ).grid(row=0, column=1, sticky="ew", padx=4, pady=2)
        ttk.Button(box, text="单次读取全部通道", command=self.single_read
                   ).grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=2)

        # ---------- 3. 数据记录 ----------
        box = ttk.LabelFrame(parent, text=" 数据记录 (CSV) ", style="Section.TLabelframe")
        box.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        box.columnconfigure(0, weight=1)
        ttk.Label(box, text="保存路径").grid(row=0, column=0, sticky="w", padx=4, pady=(2, 0))
        path_row = ttk.Frame(box)
        path_row.grid(row=1, column=0, sticky="ew", padx=4, pady=2)
        path_row.columnconfigure(0, weight=1)
        ttk.Entry(path_row, textvariable=self.dir_var).grid(row=0, column=0, sticky="ew")
        ttk.Button(path_row, text="浏览…", width=6,
                   command=self.browse_dir).grid(row=0, column=1, padx=(2, 0))
        self.record_btn = ttk.Button(box, text="开始记录", style="Big.TButton",
                                     command=self.on_record_toggle)
        self.record_btn.grid(row=2, column=0, sticky="ew", padx=4, pady=(2, 4))

        # ---------- 4. 绘图设置 ----------
        box = ttk.LabelFrame(parent, text=" 绘图设置 ", style="Section.TLabelframe")
        box.grid(row=3, column=0, sticky="ew", pady=(0, 6))
        box.columnconfigure(1, weight=1)
        self.pause_btn = ttk.Button(box, text="暂停", command=self.toggle_pause)
        self.pause_btn.grid(row=0, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(box, text="清空曲线", command=self.clear_plot
                   ).grid(row=0, column=1, sticky="ew", padx=4, pady=2)

        ttk.Label(box, text="保留点数").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        spin = ttk.Spinbox(box, from_=50, to=100000, increment=100,
                           textvariable=self.retention_var, width=8,
                           command=self.on_retention_change)
        spin.grid(row=1, column=1, sticky="w", padx=4, pady=2)
        spin.bind("<Return>", lambda _e: self.on_retention_change())
        spin.bind("<FocusOut>", lambda _e: self.on_retention_change())

        ttk.Label(box, text="超温门限").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        spin2 = ttk.Spinbox(box, from_=-200, to=1400, increment=50,
                            textvariable=self.over_temp_var, width=8,
                            command=self.on_over_temp_change)
        spin2.grid(row=2, column=1, sticky="w", padx=4, pady=2)
        spin2.bind("<Return>", lambda _e: self.on_over_temp_change())
        spin2.bind("<FocusOut>", lambda _e: self.on_over_temp_change())
        ttk.Label(box, text="°C  (K 型量程 -200 ~ 1372)").grid(row=3, column=0,
                                                              columnspan=2,
                                                              sticky="w", padx=4)
        ttk.Checkbutton(box, text="自动缩放 (取消后可鼠标缩放/平移)",
                        variable=self.autoscale_var,
                        command=self.on_autoscale_change
                        ).grid(row=4, column=0, columnspan=2, sticky="w", padx=4, pady=(2, 4))

        # ---------- 5. 运行统计 ----------
        box = ttk.LabelFrame(parent, text=" 运行统计 ", style="Section.TLabelframe")
        box.grid(row=4, column=0, sticky="ew")
        box.columnconfigure(1, weight=1)
        for row, (key, text) in enumerate((
                ("channels", "本板通道数"),
                ("fps", "帧率"),
                ("frames", "温度帧"),
                ("valid", "有效通道"),
                ("crc", "CRC 丢弃"),
                ("rows", "已记录行"),
        )):
            ttk.Label(box, text=text).grid(row=row, column=0, sticky="w", padx=4)
            var = tk.StringVar(value="-")
            self.stat_vars[key] = var
            ttk.Label(box, textvariable=var, style="Stat.TLabel"
                      ).grid(row=row, column=1, sticky="e", padx=4)

        # ---------- 6. 分度表 (TC_TABLE v1) ----------
        box = ttk.LabelFrame(parent, text=" 分度表 (TC_TABLE v1) ",
                             style="Section.TLabelframe")
        box.grid(row=5, column=0, sticky="ew", pady=(6, 0))
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="分度号").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.tc_combo = ttk.Combobox(box, textvariable=self.tc_type_var, width=5,
                                     state="readonly",
                                     values=self.tc_tables.available_types or [DEFAULT_TYPE])
        self.tc_combo.grid(row=0, column=1, sticky="w", padx=2, pady=2)
        self.tc_combo.bind("<<ComboboxSelected>>", lambda _e: self.reload_tc_table())
        ttk.Button(box, text="重载", width=5,
                   command=self.reload_tc_table).grid(row=0, column=2, padx=(2, 4), pady=2)
        ttk.Label(box, textvariable=self.tc_status_var, style="Stat.TLabel",
                  wraplength=200, justify="left"
                  ).grid(row=1, column=0, columnspan=3, sticky="w", padx=4, pady=(0, 4))

    # ------------------------------------------------------------------
    def _build_plot(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text=" 实时温度曲线 ", style="Section.TLabelframe")
        box.grid(row=0, column=0, sticky="nsew")
        box.rowconfigure(0, weight=1)
        box.columnconfigure(0, weight=1)

        self.figure = Figure(figsize=(7.6, 4.4), dpi=100, facecolor="#fcfcfc")
        self.ax = self.figure.add_subplot(111)
        self.figure.subplots_adjust(left=0.055, right=0.995, top=0.93, bottom=0.10)
        self.ax.set_facecolor("#ffffff")
        self.ax.grid(True, which="major", color="#cccccc", linewidth=0.6, alpha=0.7)
        self.ax.set_xlabel("时间 (s)")
        self.ax.set_ylabel("温度 (°C)")
        self.ax.set_title("等待数据…", fontsize=10)

        cmap = plt.get_cmap("turbo")
        # 转成 "#rrggbb": Tkinter 只认颜色名/十六进制, 不认 matplotlib 的 RGBA 元组
        self.colors = [matplotlib.colors.to_hex(cmap(i / (CHANNEL_COUNT - 1)))
                       for i in range(CHANNEL_COUNT)]
        self.lines = []
        for ch in range(CHANNEL_COUNT):
            line, = self.ax.plot([], [], lw=1.0, color=self.colors[ch],
                                 label="CH%02d" % ch, solid_joinstyle="round")
            self.lines.append(line)
        # 超温门限参考线 (虚线, 不参与图例)
        self.threshold_line = self.ax.axhline(self.over_temp, color="#ef6c00",
                                              ls="--", lw=0.9, alpha=0.75)

        self.canvas = FigureCanvasTkAgg(self.figure, master=box)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        toolbar = NavigationToolbar2Tk(self.canvas, box, pack_toolbar=False)
        toolbar.update()
        toolbar.grid(row=1, column=0, sticky="ew")
        self.canvas.draw()

        self.selector = ChannelSelector(box, colors=self.colors, columns=8,
                                        on_change=self.on_visibility_change)
        self.selector.grid(row=2, column=0, sticky="ew", padx=2, pady=(2, 2))

    # ------------------------------------------------------------------
    def _build_right_panel(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)

        box = ttk.LabelFrame(parent, text=" 实时数值 ", style="Section.TLabelframe")
        self.values_box = box          # 通道数变化时改标题用
        box.grid(row=0, column=0, sticky="nsew")
        # 4 列 x 8 行: 32 路一屏放得下, 高度也不会顶到窗口底部
        self.grid_view = ChannelGrid(box, channels=CHANNEL_COUNT, columns=4,
                                     over_temp=self.over_temp)
        self.grid_view.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        legend = ttk.Frame(box)
        legend.grid(row=1, column=0, sticky="ew", padx=4, pady=(0, 4))
        for col, (text, color) in enumerate((("正常", "#1a7f37"), ("断线", "#c62828"),
                                             ("超温", "#ef6c00"))):
            tk.Label(legend, text="● " + text, fg=color,
                     bg=legend.winfo_toplevel().cget("bg")).grid(row=0, column=col, padx=4)

        status_box = ttk.LabelFrame(parent, text=" 下位机状态 (CMD=0x11) ",
                                    style="Section.TLabelframe")
        status_box.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        status_box.columnconfigure(1, weight=1)
        rows = (("run", "运行状态"), ("dr", "实际采样率"), ("reject", "50/60 抑制"),
                ("tctable", "分度表"),
                ("rounds", "测量轮次"), ("uptime", "运行时间"), ("chip_ok", "芯片 OK 位图"),
                ("chip_err", "芯片 ERR 位图"), ("open", "断线累计"), ("spi", "SPI 失败"),
                ("timeout", "DRDY 超时"), ("flags", "异常标志"), ("age", "状态帧龄期"))
        for row, (key, text) in enumerate(rows):
            ttk.Label(status_box, text=text).grid(row=row, column=0, sticky="w", padx=4)
            var = tk.StringVar(value="-")
            self.stat_vars[key] = var
            ttk.Label(status_box, textvariable=var, style="Stat.TLabel"
                      ).grid(row=row, column=1, sticky="e", padx=4)

    # ------------------------------------------------------------------
    def _build_statusbar(self) -> None:
        bar = ttk.Frame(self.root, padding=(6, 2))
        bar.grid(row=1, column=0, sticky="ew")
        bar.columnconfigure(0, weight=1)
        self.status_label = tk.Label(bar, textvariable=self.status_var, anchor="w",
                                     font=("Microsoft YaHei", 9))
        self.status_label.grid(row=0, column=0, sticky="ew")
        tk.Label(bar, textvariable=self.counters_var, anchor="e",
                 font=("Consolas", 9)).grid(row=0, column=1, sticky="e")

    def _build_menu(self) -> None:
        menubar = tk.Menu(self.root)
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="选择保存路径…", command=self.browse_dir)
        file_menu.add_command(label="打开数据目录", command=self.open_data_dir)
        file_menu.add_separator()
        file_menu.add_command(label="退出", command=self.on_close)
        menubar.add_cascade(label="文件", menu=file_menu)

        tool_menu = tk.Menu(menubar, tearoff=0)
        tool_menu.add_command(label="查看日志窗口", command=self.open_log_window)
        tool_menu.add_command(label="协议自检", command=self.run_selftest)
        tool_menu.add_separator()
        tool_menu.add_command(label="清空曲线", command=self.clear_plot)
        menubar.add_cascade(label="工具", menu=tool_menu)

        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="使用说明", command=self.show_help)
        menubar.add_cascade(label="帮助", menu=help_menu)
        self.root.configure(menu=menubar)

    def _bind_shortcuts(self) -> None:
        self.root.bind("<Control-r>", lambda _e: self.on_record_toggle())
        self.root.bind("<Control-p>", lambda _e: self.toggle_pause())
        self.root.bind("<Control-l>", lambda _e: self.open_log_window())
        self.root.bind("<F5>", lambda _e: self.refresh_ports())

    # ==================================================================
    # 串口 / 连接
    # ==================================================================
    def refresh_ports(self) -> None:
        """扫描串口并刷新下拉框 (保留当前选择)。"""
        current = self.port_var.get()
        ports = available_ports()
        self.port_combo.configure(values=ports)
        if root_ports := ports:
            if current not in ports:
                self.port_var.set(root_ports[0])
        else:
            self.port_var.set("")
        self._log("扫描到 %d 个串口: %s" % (len(ports), ", ".join(ports) if ports else "无"))

    def on_connect_toggle(self) -> None:
        if self.reader is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> None:
        """打开串口 / 启动演示源, 并自动下发采样率与"启动采集"。"""
        if self.reader is not None:
            return
        port = (port or self.port_var.get()).strip()
        baud = int(baudrate or self.baud_var.get())
        if self.demo:
            port = "DEMO (虚拟下位机)"      # 演示模式不使用真实串口
        elif not port:
            messagebox.showwarning("提示", "没有可用串口。请插入设备后点击“刷新”，\n"
                                           "或使用 --demo 参数体验演示模式。")
            return

        # 新会话: 清空残留消息与曲线数据
        self.rx_queue = queue.Queue()
        self.ring.clear()
        self.latest = np.full(CHANNEL_COUNT, np.nan, dtype=float)
        self.t0 = None
        self.temp_frames = 0
        self.crc_errors = 0
        self.unknown_frames = 0
        self.last_temp_time = 0.0
        self._frame_times.clear()
        self._timeout_warned = False
        self.grid_view.reset()
        self._plot_dirty = True
        self._values_dirty = True

        if self.demo:
            # 演示模式的片数跟随 --channels (默认 4 片 = 8 路)
            self.reader = DemoSource(self.rx_queue,
                                     chip_count=(self.forced_channels // 2)
                                     if self.forced_channels else 4)
            self._log("已连接虚拟下位机 (DEMO)")
        else:
            self.reader = SerialReader(port, baud, self.rx_queue,
                                       reconnect=True, reconnect_interval=1.0)
            self._log("正在打开串口 %s @ %d ..." % (port, baud))
        self.reader.start()

        self.connect_btn.configure(text="断开")
        self.port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self.ax.set_title("%s  —  %s @ %d bps" %
                          (APP_TITLE.split("·")[0].strip(),
                           port or "DEMO", baud), fontsize=10)
        # 等串口稳定后再下发配置
        self.root.after(400, self._post_connect_config)

    def _post_connect_config(self) -> None:
        """连接成功后的默认动作: 设置采样率 + 启动采集。"""
        if self.reader is None:
            return
        self.apply_rate(from_user=False)
        self.acquire(True, from_user=False)

    def disconnect(self) -> None:
        """停止串口线程并复位界面状态。"""
        reader, self.reader = self.reader, None
        if reader is not None:
            reader.stop()
            self._log("串口已断开")
        if self.recorder.active:
            self.stop_recording()
        self.connect_btn.configure(text="连接")
        self.port_combo.configure(state="readonly")
        self.baud_combo.configure(state="readonly")
        self.lamp_conn.set_state("off", "未连接")
        self.lamp_rec.set_state("off", "未记录")
        self.ax.set_title("等待数据…", fontsize=10)

    # ==================================================================
    # 下行命令
    # ==================================================================
    def _send_frame(self, frame: bytes, desc: str) -> bool:
        if self.reader is None:
            self._log("未连接, 命令未发送: %s" % desc, "warn")
            return False
        self.reader.send(frame)
        self._log("发送 %s: %s" % (desc, frame.hex(" ").upper()))
        return True

    def apply_rate(self, from_user: bool = True) -> None:
        """把界面上的采样率/抑制设置下发 (CMD=0x01)。"""
        try:
            sps = int(str(self.rate_var.get()).split()[0])
        except (ValueError, IndexError):
            return
        reject = REJECT_TABLE.index(self.reject_var.get())
        if sps != 20 and reject != 0:
            self._log("注意: 50/60Hz 抑制只在 20SPS 下有效, 固件会自动关闭该抑制", "warn")
        advice = baudrate_advice(sps)
        if advice and int(self.baud_var.get()) < 460800:
            self._log("警告: " + advice, "warn")
        if self.reader is not None:
            try:
                frame = cmd_set_rate_sps(sps, reject)
            except ValueError as exc:
                self._log(str(exc), "error")
                return
            self._send_frame(frame, "设置采样率 %d SPS / %s" % (sps, self.reject_var.get()))
        elif from_user:
            self._log("采样率已选择 %d SPS (连接后自动下发)" % sps)

    def acquire(self, start: bool, from_user: bool = True) -> None:
        """启动/停止下位机采集 (CMD=0x02)。"""
        if self.reader is None:
            if from_user:
                self._log("未连接, 无法%s采集" % ("启动" if start else "停止"), "warn")
            return
        self._send_frame(cmd_run(start), "%s采集" % ("启动" if start else "停止"))

    def single_read(self) -> None:
        """读取单次温度 (CMD=0x03, 全部通道)。"""
        if self.reader is None:
            self._log("未连接, 无法单次读取", "warn")
            return
        self._single_pending = time.time()
        self._send_frame(cmd_single(), "单次读取全部通道")

    # ==================================================================
    # 记录
    # ==================================================================
    def browse_dir(self) -> None:
        initial = self.dir_var.get() or "."
        chosen = filedialog.askdirectory(title="选择 CSV 保存文件夹",
                                         initialdir=initial if os.path.isdir(initial) else ".")
        if chosen:
            self.dir_var.set(os.path.abspath(chosen))
            self._log("保存路径: %s" % self.dir_var.get())

    def open_data_dir(self) -> None:
        path = self.dir_var.get()
        if os.path.isdir(path):
            try:
                os.startfile(path)            # Windows
            except AttributeError:            # pragma: no cover - 非 Windows
                self._log("请手动打开: %s" % path)
        else:
            self._log("目录不存在: %s" % path, "warn")

    def on_record_toggle(self) -> None:
        if self.recorder.active:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self) -> None:
        try:
            path = self.recorder.start_recording(self.dir_var.get() or "./data")
        except Exception as exc:
            messagebox.showerror("无法开始记录", str(exc))
            self._log("开始记录失败: %s" % exc, "error")
            return
        self.dir_var.set(os.path.dirname(path))
        self.record_rows_base = 0
        self.record_btn.configure(text="停止记录")
        self._log("开始记录 -> %s" % path)

    def stop_recording(self) -> None:
        info = self.recorder.stop_recording()
        self.record_btn.configure(text="开始记录")
        self.lamp_rec.set_state("off", "未记录")
        message = ("停止记录: %d 行 -> %s" % (info["rows"], info["temp_path"]))
        if info["dropped"]:
            message += " (队列溢出丢弃 %d 行)" % info["dropped"]
        if info["error"]:
            message += " [写入错误: %s]" % info["error"]
            self._log(message, "error")
        else:
            self._log(message)

    # ==================================================================
    # 绘图参数
    # ==================================================================
    def on_retention_change(self) -> None:
        try:
            value = int(self.retention_var.get())
        except (tk.TclError, ValueError):
            return
        value = max(50, min(value, 100000))
        self.retention_var.set(value)
        self.ring.resize(value)
        self._plot_dirty = True
        self._log("保留点数 = %d" % value)

    def on_over_temp_change(self) -> None:
        try:
            value = float(self.over_temp_var.get())
        except (tk.TclError, ValueError):
            return
        self.over_temp = value
        self.grid_view.set_over_temp(value)
        self.threshold_line.set_ydata([value, value])
        self._plot_dirty = True
        self._values_dirty = True
        self._log("超温门限 = %.1f °C" % value)

    def on_autoscale_change(self) -> None:
        self._plot_dirty = True
        self._log("自动缩放: %s" % ("开" if self.autoscale_var.get() else "关 (可鼠标缩放/平移)"))

    def on_visibility_change(self, mask: np.ndarray) -> None:
        self.visible = mask
        self._plot_dirty = True
        self._values_dirty = True

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_btn.configure(text="继续" if self.paused else "暂停")
        self._log("曲线%s (数据仍在接收/记录)" % ("已暂停" if self.paused else "已继续"))
        self._plot_dirty = True

    def clear_plot(self) -> None:
        self.ring.clear()
        self.t0 = None
        for line in self.lines:
            line.set_data([], [])
        self._plot_dirty = True
        self._log("曲线已清空 (记录不受影响)")

    # ==================================================================
    # 队列消息处理 (全部在 UI 线程执行)
    # ==================================================================
    def _handle_message(self, msg: Dict) -> None:
        kind = msg.get("type")
        if kind == "temp":
            self._on_temp(msg)
        elif kind == "raw":
            self._on_raw(msg)
        elif kind == "status":
            self._on_status(msg)
        elif kind == "connected":
            self.lamp_conn.set_state("ok", "已连接")
            self._log("串口已连接: %s @ %s" % (msg.get("port"), msg.get("baudrate")))
        elif kind == "disconnected":
            self.lamp_conn.set_state("error", "已断开")
            self._log("串口断开: %s (线程将自动重连)" % msg.get("reason"), "error")
        elif kind == "reconnecting":
            self.lamp_conn.set_state("warn", "重连中")
            if msg.get("attempt", 0) <= 3:
                self._log("重连中 (第 %d 次): %s" % (msg.get("attempt"), msg.get("reason")), "warn")
        elif kind == "crc_error":
            self.crc_errors = int(msg.get("count", 0))
            self._log("CRC 校验失败, 已丢弃帧 (累计 %d)" % self.crc_errors, "warn")
        elif kind == "unknown":
            self.unknown_frames += 1
            if time.time() - self.last_unknown_time > 2.0:
                self.last_unknown_time = time.time()
                self._log("收到未知上行帧 CMD=0x%02X LEN=%d (协议不匹配?)"
                          % (msg.get("cmd", 0), msg.get("length", 0)), "warn")
        elif kind == "error":
            self._log(msg.get("message", "未知错误"), "error")
        elif kind == "info":
            self._log(msg.get("message", ""))

    def _on_raw(self, msg: Dict) -> None:
        """★ CMD=0x12 原始帧: 用当前分度表把 µV 算成 °C, 再走温度帧同一条通路。

        这就是"温度由上位机换算": 固件只给"实测热电势 µV + 冷端温度 °C",
        这里用 tables/tc_type_<x>.csv + 单调三次插值(pchip) 算出热端温度。

        好处: 换分度号 (K/J/T/E/N) / 换厂家标定表 只需要在界面上切一下,
              完全不用重新编译下载固件; CSV 里同时存了 µV, 换表还能离线重算。
        """
        now = msg.get("t", time.time())
        self.raw_frames += 1

        table = self.tc_table
        if table is None:
            if not self._raw_no_table_warned:
                self._raw_no_table_warned = True
                self._log("收到原始帧 (CMD=0x12), 但分度表没加载成功, 无法换算温度。"
                          "请检查 tables/ 目录 (可用 python tools/nist_tc_tables.py 生成)",
                          "error")
            return

        try:
            emf = np.asarray(msg.get("emf_uv", []), dtype=float)
            if emf.size < CHANNEL_COUNT:
                emf = np.concatenate([emf,
                                      np.full(CHANNEL_COUNT - emf.size, np.nan)])
            # 冷端是"每片一个", 先展开成"每通道一个"再补偿
            cj = np.asarray(cj_per_channel(list(msg.get("cj_c", []))), dtype=float)
            temps = np.asarray(table.compensate_hot_junction(emf, cj, MODE_PCHIP),
                               dtype=float)
        except Exception as exc:                     # 表坏了/数值异常都不该让 UI 崩
            if not self._raw_conv_warned:
                self._raw_conv_warned = True
                self._log("原始帧温度换算失败 (%s型): %s"
                          % (getattr(table, "tc_type", "?"), exc), "error")
            return

        # 超出分度表范围 / 断线 -> NaN; 其余保留
        temps = np.where(np.isfinite(temps), temps, np.nan)
        self._last_emf_uv = emf
        self._on_temp({"t": now, "temps": temps.tolist()}, emf, msg.get("cj_c"))

    def _on_temp(self, msg: Dict, emf_uv=None, cj_c=None) -> None:
        now = msg.get("t", time.time())
        values = np.asarray(msg.get("temps", []), dtype=float)
        if values.size < CHANNEL_COUNT:
            values = np.concatenate([values, np.full(CHANNEL_COUNT - values.size, np.nan)])
        self.latest = values
        self.temp_frames += 1
        self.last_temp_time = now
        self._frame_times.append(now)
        self._values_dirty = True
        self._timeout_warned = False

        # CMD=0x03 的单次读取: 只更新数值面板, 不进曲线也不记录 (避免把曲线打断)
        if self._single_pending and (now - self._single_pending) < 2.0:
            self._single_pending = 0.0
            valid = np.count_nonzero(~np.isnan(values))
            self._log("单次读取结果: %d/32 路有效" % valid)
            return

        if self.t0 is None:
            self.t0 = now
        self.ring.push(now, values)
        self._plot_dirty = True
        if self.recorder.active:
            # ★ 原始模式下把 µV 和冷端也一起存进 CSV:
            #   有了这两个量, 以后换分度表就能**离线重算**历史数据。
            self.recorder.submit(now, values, emf_uv, cj_c)

    def _on_status(self, msg: Dict) -> None:
        status = msg.get("status") or {}
        self.status = status
        self.last_status_time = msg.get("t", time.time())
        if "error" in status:
            self._log("状态帧解析失败: %s" % status["error"], "error")
            return

        # ★ 固件处于"原始上报"模式吗? (发 CMD=0x12 的 µV+冷端, 温度由我们自己算)
        #   只在状态变化时提示一次, 免得每 5 秒刷一条日志。
        is_raw = bool(int(status.get("flags") or 0) & ST_FLAG_RAW_UPLINK)
        if is_raw != self.raw_mode or not self._raw_mode_logged:
            self.raw_mode = is_raw
            self._raw_mode_logged = True
            self._log("固件上报模式: %s" % (
                "原始帧 CMD=0x12 (µV + 冷端 °C, 温度由上位机查分度表算)" if is_raw
                else "温度帧 CMD=0x10 (固件已算好 °C)"))

        # ★ 跟随固件上报的本板片数: 8 路板就只显示 8 路, 不再把协议里
        #   那 24 个恒为 NaN 的槽位显示成"断线"。
        if not self.forced_channels:
            active = int(status.get("active_channels") or CHANNEL_COUNT)
            if active != self.active_channels:
                self._apply_active_channels(active)

        if self.recorder.active:
            self.recorder.submit_status(self.last_status_time, status)

    # ==================================================================
    # 分度表 (TC_TABLE v1)
    # ==================================================================
    def reload_tc_table(self) -> None:
        """加载当前选中的分度表 (默认 K 型) 并刷新界面状态。

        表文件来自工程 tables/ 目录 (由 tools/nist_tc_tables.py 生成, 带 sha256)。
        加载时会校验单位/单调性/哈希, 任何一项不过就报错而不是"带病工作"。
        """
        tc = (self.tc_type_var.get() or DEFAULT_TYPE).upper()
        table = self.tc_tables.try_get(tc)
        if table is None:
            self.tc_table = None
            msg = self.tc_tables.errors.get(tc, "加载失败")
            self.tc_status_var.set("✗ %s" % msg)
            self.stat_vars["tctable"].set("-")
            self._log("分度表 %s 型加载失败: %s" % (tc, msg), "error")
            return
        self.tc_table = table
        sha = str(table.meta.get("sha256", ""))[:8]
        self.tc_status_var.set("%d 点 · %.0f…%.0f °C · sha %s"
                               % (table.points, table.temp_min, table.temp_max, sha))
        self.stat_vars["tctable"].set("%s 型 · %d 点" % (table.tc_type, table.points))
        self._log("分度表已加载: %s   sha256 %s…" % (table.describe(), sha))

    def _apply_active_channels(self, count: int) -> None:
        """按本板实际通道数调整数值面板 / 勾选框 / 曲线。"""
        count = max(2, min(int(count), CHANNEL_COUNT))
        if count == self.active_channels:
            return
        self.active_channels = count
        self.grid_view.set_active_channels(count)
        self.selector.set_active_channels(count)
        self.values_box.configure(text=" 实时数值 (%d 路) " % count)
        self.visible = self.selector.visible_mask()
        for ch in range(count, CHANNEL_COUNT):
            self.lines[ch].set_visible(False)
        self._plot_dirty = True
        self._values_dirty = True
        self._log("识别到本板通道数: %d 路 (%d 片 ADS1220)"
                  % (count, (count + 1) // 2))

    # ==================================================================
    # 定时刷新
    # ==================================================================
    def _poll(self) -> None:
        if self._closing:
            return
        for _ in range(self.MAX_MSGS_PER_TICK):
            try:
                msg = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_message(msg)

        self._refresh_lamps()
        self._watchdog()
        if self._plot_dirty and not self.paused:
            self._update_plot()
        if self._values_dirty:
            self._update_values()
        self._update_stats()
        self.root.after(self.POLL_MS, self._poll)

    def _refresh_lamps(self) -> None:
        # 连接灯
        if self.reader is None:
            self.lamp_conn.set_state("off", "未连接")
        elif getattr(self.reader, "is_connected", False):
            self.lamp_conn.set_state("ok", "已连接")
        else:
            self.lamp_conn.set_state("warn", "重连中")
        # 记录灯: 红灯闪烁表示正在写入
        if self.recorder.active:
            blink = int(time.time() * 2) % 2 == 0
            self.lamp_rec.set_state("rec" if blink else "rec_dim",
                                    "记录中 %d" % self.recorder.rows_written)
        else:
            self.lamp_rec.set_state("off", "未记录")

    def _watchdog(self) -> None:
        """超过 DATA_TIMEOUT_S 没有温度帧就提示"下位机断线"。"""
        now = time.time()
        if self.reader is None or not getattr(self.reader, "is_connected", False):
            return
        if self.last_temp_time == 0.0:
            return
        gap = now - self.last_temp_time
        if gap > self.DATA_TIMEOUT_S and not self._timeout_warned:
            self._timeout_warned = True
            self._log("已 %.1fs 未收到温度帧 —— 请检查下位机是否在运行 / 波特率是否匹配" % gap,
                      "error")

    def _update_values(self) -> None:
        self.grid_view.update_values(self.latest)
        self._values_dirty = False

    def _update_plot(self) -> None:
        if self.ring.count == 0:
            self._plot_dirty = False
            return
        t, data = self.ring.snapshot()
        origin = self.t0 if self.t0 is not None else t[0]
        x = t - origin

        for ch, line in enumerate(self.lines):
            if self.visible[ch] and ch < self.active_channels:
                line.set_visible(True)
                line.set_data(x, data[:, ch])
            else:
                line.set_visible(False)

        if self.autoscale_var.get():
            if len(x) >= 2:
                self.ax.set_xlim(float(x[0]), float(max(x[-1], x[0] + 1e-3)))
            # 自动缩放只看本板真实存在、且被勾选的那些通道
            mask = self.visible & (np.arange(CHANNEL_COUNT) < self.active_channels)
            if mask.any():
                block = data[:, mask]
                if block.size:
                    vmin = float(np.nanmin(block))
                    vmax = float(np.nanmax(block))
                    if math.isfinite(vmin) and math.isfinite(vmax):
                        pad = max(1.0, (vmax - vmin) * 0.08)
                        self.ax.set_ylim(vmin - pad, vmax + pad)
        self.canvas.draw_idle()
        self._plot_dirty = False

    def _update_stats(self) -> None:
        # 帧率: 统计最近 2 秒内的温度帧
        now = time.time()
        while self._frame_times and now - self._frame_times[0] > 2.0:
            self._frame_times.popleft()
        fps = len(self._frame_times) / 2.0 if self._frame_times else 0.0
        # 有效通道只统计本板真实存在的那些 (协议里没接的槽位恒为 NaN)
        valid = int(np.count_nonzero(~np.isnan(self.latest[:self.active_channels])))
        self.stat_vars["channels"].set("%d 路 / %d 片" % (self.active_channels,
                                                          (self.active_channels + 1) // 2))
        self.stat_vars["fps"].set("%.1f Hz" % fps)
        self.stat_vars["frames"].set("%d 帧" % self.temp_frames)
        self.stat_vars["valid"].set("%d / %d" % (valid, self.active_channels))
        self.stat_vars["crc"].set("%d 帧" % self.crc_errors)
        self.stat_vars["rows"].set("%d 行" % self.recorder.rows_written)

        status = self.status or {}
        self.stat_vars["run"].set("运行" if status.get("run") else
                                  ("停止" if status else "-"))
        self.stat_vars["dr"].set("%s SPS" % (status.get("dr_sps") or "-"))
        self.stat_vars["reject"].set(str(status.get("reject", "-")))
        self.stat_vars["rounds"].set(str(status.get("rounds", "-")))
        self.stat_vars["uptime"].set("%.1f s" % (status.get("uptime_ms", 0) / 1000.0)
                                     if status else "-")
        # 位图只显示本板实际片数那几位 (8 路板就是 4 位, 不会出现一堆前导 0)
        bits = max(1, (self.active_channels + 1) // 2)
        self.stat_vars["chip_ok"].set(str(status.get("chip_ok_text", "-"))[-bits:])
        self.stat_vars["chip_err"].set(str(status.get("chip_err_text", "-"))[-bits:])
        self.stat_vars["open"].set(str(status.get("open_cnt", "-")))
        self.stat_vars["spi"].set(str(status.get("spi_err", "-")))
        self.stat_vars["timeout"].set(str(status.get("timeout_cnt", "-")))
        self.stat_vars["flags"].set(str(status.get("flag_text", "-")))
        age = "%0.1f s" % (now - self.last_status_time) if self.last_status_time else "-"
        self.stat_vars["age"].set(age)

        self.counters_var.set(
            "本板 %d 路 | 帧率 %.1f Hz | 温度帧 %d | CRC 丢弃 %d | 有效 %d/%d | 记录 %d 行%s"
            % (self.active_channels, fps, self.temp_frames, self.crc_errors, valid,
               self.active_channels, self.recorder.rows_written,
               " | 曲线暂停" if self.paused else ""))

    # ==================================================================
    # 日志 / 辅助窗口
    # ==================================================================
    def _log(self, message: str, level: str = "info") -> None:
        """底部状态栏 + 日志窗口 (时间戳, 只保留最近 800 条) + 文件日志。"""
        line = "[%s] %s" % (time.strftime("%H:%M:%S"), message)
        self.log_lines.append(line)
        self.file_logger.write(message, level)
        colors = {"info": "#111111", "warn": "#b26a00", "error": "#c62828"}
        self.status_var.set(message)
        try:
            self.status_label.configure(fg=colors.get(level, "#111111"))
        except tk.TclError:               # pragma: no cover
            pass
        if self.log_window is not None:
            try:
                self.log_window.text.configure(state="normal")
                self.log_window.text.insert("end", line + "\n")
                self.log_window.text.see("end")
                self.log_window.text.configure(state="disabled")
            except tk.TclError:           # pragma: no cover
                self.log_window = None

    def open_log_window(self) -> None:
        if self.log_window is not None:
            self.log_window.lift()
            return
        win = tk.Toplevel(self.root)
        win.title("运行日志")
        win.geometry("760x320")
        text = tk.Text(win, wrap="none", font=("Consolas", 9))
        text.pack(fill="both", expand=True)
        text.insert("end", "\n".join(self.log_lines) + "\n")
        text.configure(state="disabled")
        win.text = text                     # 便于 _log 追加
        win.protocol("WM_DELETE_WINDOW", lambda: self._close_log_window(win))
        self.log_window = win

    def _close_log_window(self, win: tk.Toplevel) -> None:
        self.log_window = None
        win.destroy()

    def run_selftest(self) -> None:
        """在界面里跑一遍协议自检 (CRC/组帧/拆帧)。"""
        try:
            from .protocol import selftest
        except ImportError:               # pragma: no cover
            from protocol import selftest
        if selftest(verbose=False):
            self._log("协议自检: 全部通过")
            messagebox.showinfo("协议自检", "CRC-16/MODBUS、组帧、拆帧、抗干扰、状态解析 — 全部通过")
        else:
            self._log("协议自检: 存在失败项", "error")
            messagebox.showerror("协议自检", "自检失败, 详见命令行输出")

    def show_help(self) -> None:
        messagebox.showinfo("使用说明", (
            "1) 选择串口与波特率 (600SPS 建议 460800), 点“连接”;\n"
            "2) 连接后自动下发采样率(CMD=0x01)并启动采集(CMD=0x02);\n"
            "3) 点“开始记录”写入 CSV (temperature_日期_时间.csv);\n"
            "4) 中部曲线: 勾选框控制 32 路的显示/隐藏, 可暂停;\n"
            "5) 右侧网格: 每路温度与状态(正常/断线/超温);\n"
            "6) 快捷键: Ctrl+R 记录, Ctrl+P 暂停, Ctrl+L 日志, F5 刷新串口。\n\n"
            "数据格式: 0x55/0xAA + CMD + LEN + DATA + CRC16/MODBUS(小端)"
        ))

    # ==================================================================
    # 收尾
    # ==================================================================
    def on_close(self) -> None:
        if self._closing:
            return
        if self.recorder.active:
            if not messagebox.askokcancel("退出", "正在记录数据, 确定退出并保存吗?"):
                return
        self._closing = True
        try:
            if self.reader is not None:
                self.reader.stop()
            if self.recorder.active:
                info = self.recorder.stop_recording()
                print("已保存 %d 行 -> %s" % (info["rows"], info["temp_path"]))
        finally:
            self.file_logger.close()
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(argv: Optional[List[str]] = None) -> int:
    """供 `python -m pc_ui.main` 使用的最简入口 (完整参数见 pc_ui/main.py)。"""
    try:
        from .main import main as _main
    except ImportError:                   # pragma: no cover
        from main import main as _main
    return _main(argv)


if __name__ == "__main__":                # pragma: no cover
    sys.exit(main())
