# tools/serial_sim —— 虚拟串口联调工具

不用板子，就能让**真实的上位机**（`pc_ui`）连上"固件"，收数据、算温度、看曲线。

这里有两个东西：

| 脚本 | 作用 |
| --- | --- |
| [`sim_device.py`](sim_device.py) | **模拟下位机**：在 COM 口上扮演 STM32F103 + 4×ADS1220，协议与固件一字不差 |
| [`test_loopback.py`](test_loopback.py) | **回环自检**：模拟器走 COM2，真上位机读取线程走 COM1，检查能不能收到并算出温度 |

协议/温度换算都复用 `pc_ui/protocol.py` 和 `pc_ui/tc_table.py`，
所以"模拟器算出来的 µV"和"上位机算回去的温度"必然闭环——
这正是要验证的东西。

---

## 1. 先建一对虚拟串口

用 **ELTIMA Virtual Serial Port** 或 **com0com** 建一对互联的口，例如：

```
COM1 <----虚拟线----> COM2
```

然后约定：**COM1 给上位机，COM2 给模拟器**（反过来也行，用 `--port` 指定）。

看当前有哪些口：

```powershell
python -m tools.serial_sim.sim_device --list-ports
```

---

## 2. 三个常用命令

### ① 先自检（不需要串口，几秒钟）

```powershell
python -m tools.serial_sim.sim_device --test
```

会校验：分度表反算闭环（-100…1300 °C 误差全为 0）、原始帧字节布局（197 字节 / LEN=192）、
状态帧字段（chip_count=4 / RAW 标志位）、下行命令解析（采样率/启停/单次/已移位写法）。

### ② 手动跑模拟器（自己另开一个终端）

```powershell
# 模拟器占 COM2，默认 4 片 = 8 路、20 SPS、上行 0x12 原始帧
python -m tools.serial_sim.sim_device --port COM2
```

然后上位机选 COM1、115200 连上去就能看到曲线。

### ③ 一条命令跑完整回环（推荐，先跑这个）

```powershell
python -m tools.serial_sim.test_loopback --duration 7
```

自动把 COM1 分给上位机读取线程、COM2 分给模拟器，跑 7 秒，检查：

- [x] 真上位机的 `SerialReader` 能收到 `CMD=0x12` 原始帧
- [x] 原始帧字段完整（µV 32 路 + 冷端 16 路）
- [x] 能用 `tables/tc_type_k.csv` 换算成温度（有限值、范围合理）
- [x] 状态帧 `chip_count=4` / `active_channels=8` / `RAW_UPLINK` 标志置位
- [x] 没有未知帧（说明协议对得上）

---

## 3. 参数速查

```powershell
python -m tools.serial_sim.sim_device --port COM2 \
    --chips 4 \          # ADS1220 片数 (1..8)，4 = 8 路
    --sps 20 \           # 20/45/90/175/330/600/1000
    --uplink raw \       # raw=0x12(默认)  temp=0x10  both=两帧都发
    --corrupt 0 \        # 坏帧概率，用来测上位机的 CRC 丢帧计数
    --seed 1 \           # 固定随机种子，数据可复现
    --quiet              # 只输出关键事件
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--port` | （无） | 串口名；不给就进 dry-run 只打印帧 |
| `--chips` | `4` | 虚拟 ADS1220 片数，决定路数（×2） |
| `--sps` | `20` | 采样率；一轮 = 3 个转换阶段，所以帧率 ≈ sps/3 |
| `--uplink` | `raw` | 对应固件 `TC_UPLINK_MODE`：`raw`=1 / `temp`=0 / `both`=2 |
| `--no-autorun` | 关 | 不自动启动采集，等上位机发 `0x02` |
| `--corrupt` | `0` | 坏帧概率（0..1），默认全干净 |
| `--seed` | 随机 | 固定后每次演示数据一致 |
| `--dry-run` | 关 | 不占串口，只打印每帧的字节 |
| `--test` | — | 只跑自检然后退出 |

---

## 4. 和固件的对应关系

模拟器严格按 `Core/Src/main.c` 的状态机走：

| 固件 | 模拟器 |
| --- | --- |
| `App_HwInit()` → 上电复位/回读校验/失调校准 | 直接加载分度表，上电即 `App_Start()` |
| `App_Start()` / `App_Stop()` | 处理 `0x02` 命令 |
| `App_SetRate()` | 处理 `0x01` 命令（DR + 50/60 抑制，含"已移位"写法） |
| `App_RequestSingle()` | 处理 `0x03` 命令 |
| `App_ComputeTemps()` | `ChannelModel.hot_temps()` + 分度表反算成 µV |
| `App_ReportTemps()` | `build_raw_frame()` / `build_temp_frame()` |
| `App_ReportStatus()` | `_build_status_payload()`（每 5 s + 命令应答） |
| 一轮 = A 通道 → B 通道 → 冷端（各 1 个转换周期） | `_round_period() = 3 / sps` |

演示数据里带了几个**故障注入**，方便看状态列和曲线的反应：

- 最后一路周期性断线（NaN）→ 上位机显示"断线"，`open_tc_count` 累加
- 第 4 路稳态 ~1055 °C → 超温告警（默认门限 1000 °C）
- 每通道 ~0.08 °C 高斯噪声 + 0.05 Hz 慢漂移
- 每片冷端温度 25 °C + 0.5 °C×片号（模拟等温块梯度）

---

## 5. 常见问题

**`could not open port 'COMx': Access denied`**

三种可能，按顺序排查：

1. **口被别的程序占着**（上位机自己、上一次没退干净的模拟器）——任务管理器里看有没有残留的 `python` 进程，有的话结束掉。
2. **上一次测试被强杀，ELTIMA 驱动状态脏了**（端口列表里还看得见，但打不开）——这最常见。到 ELTIMA 面板把这一对虚拟串口**断开再连上**，或者干脆重启虚拟串口驱动。我这次就是强杀后台任务后踩到的。
3. **虚拟串口对还没建好**——`--list-ports` 确认至少有 2 个口。

`test_loopback.py` 里已经加了 6 次重试（每次隔 0.5 s），但驱动状态脏了的话重试也没用，得先按第 2 条处理。

**`没找到虚拟串口对`**

`--list-ports` 看看有几个口。只有 1 个说明虚拟串口对还没建好。

**收到原始帧但温度算不出来（显示 NaN）**

`tables/` 目录缺失或分度表 sha256 校验不过。先跑：

```powershell
python tools/nist_tc_tables.py          # 重新生成 tables/*.csv
python -m pc_ui.table_selftest          # 校验
```

或者用 `--uplink temp` 让模拟器直接发 `0x10` 温度帧绕过。

**想测上位机的 CRC 丢帧**

`--corrupt 0.05` 注入 5% 坏帧，看上位机状态栏的 CRC 错误计数。
