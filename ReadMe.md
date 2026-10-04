# STM32F103C8T6 · 4 × ADS1220 · 8 路 K 型热电偶测温固件

> 从原来的 **4 路 MAX31855** 模板改造而来：芯片换成了 **ADS1220**，
> 片数从 4 片缩到 **4 片**（原 16 片方案已改为 4 片，见 `docs/SCHEMATIC_CHANGES.md`），
> 通道从 4 路扩到 **8 路**，
> 片选用 **PB0~PB3**，DRDY 是 **每片一根（PA0~PA3）直接进 MCU 轮询**，
> 不再用 74HC30/74HC132 合并、也不再用 EXTI 中断。
> 换芯片后必须改的那些细节（SPI Mode 0→1、RXONLY→全双工、
> 时钟分频、寄存器位位置、冷端补偿算法、协议帧）全部已处理，
> 并在 `docs/TIMING.md` §10 逐条对照说明。
>
> 协议温度帧固定 32 槽（`docs` 里的"32 路"指协议槽位/协议帧，
> **本板实际是 8 路**；没接的槽位填 NaN，上位机自动跟随状态帧的片数显示）。

---

## 1. 硬件连接

### 1.1 MCU 引脚分配（LQFP48，15/35 个 IO 被占用）

| 引脚 | 功能 | 去向 |
|---|---|---|
| `PA5` | SPI1_SCK | 4 片 ADS1220 的 SCLK（并联） |
| `PA6` | SPI1_MISO | 4 片的 DOUT/DRDY（并联，靠片选三态隔离） |
| `PA7` | SPI1_MOSI | 4 片的 DIN（并联） |
| `PA0`~`PA3` | **DRDY0 ~ DRDY3** | 第 0~3 片 ADS1220 的专用 DRDY 引脚（上拉输入，主循环轮询） |
| `PA9` / `PA10` | USART1_TX / RX | PC 上位机，115200（可 460800）8N1 |
| `PA13` / `PA14` | SWDIO / SWCLK | 调试器（注意：**不要用 JTAG**，PA15/PB3/PB4 被 SWJ 默认占用，需关 JTAG 才能用） |
| `PB0`~`PB3` | **CS0 ~ CS3** | 第 0~3 片 ADS1220 的 CS（推挽 50 MHz，初始高电平） |
| 空闲 | 其余 20 个 IO | 可留作指示灯/蜂鸣器/预留 |

### 1.2 通道映射

```
ADS1220 #0  AIN0/AIN1 -> ch0     AIN2/AIN3 -> ch1
ADS1220 #1  AIN0/AIN1 -> ch2     AIN2/AIN3 -> ch3
ADS1220 #2  AIN0/AIN1 -> ch4     AIN2/AIN3 -> ch5
ADS1220 #3  AIN0/AIN1 -> ch6     AIN2/AIN3 -> ch7
```
即 `ch[2n]` = 第 n 片 A 通道（MUX = `0000b`），`ch[2n+1]` = 第 n 片 B 通道（MUX = `0101b`）。

### 1.3 DRDY：每片一根，主循环轮询

```
ADS1220 #0 DRDY ──► PA0 (上拉输入)
ADS1220 #1 DRDY ──► PA1
ADS1220 #2 DRDY ──► PA2
ADS1220 #3 DRDY ──► PA3
```

要点：

* ADS1220 的 DRDY 是**低有效、主动推挽输出**，而且"CS 为高时也一直驱动"
  （手册 8.5.1.3），所以不需要上拉也能读到，固件仍配了内部上拉防悬空。
* DRDY 会在**下一个 SCLK 上升沿**自动回到高电平 → **读完数据 DRDY 自己就恢复了，
  不需要任何"复位"操作**。
* 每片一根线，**能分辨是哪一片没就绪**：等待窗口里逐片查 DRDY 电平，
  全部为低才继续；哪片一直拉高，`flags` 会置 `DRDY_PARTIAL(0x20)`、
  `chip_err` 位图里对应位会置起来（老 16 片方案的合并信号做不到这一点）。
* 因为没有了 EXTI 中断，等待"全部就绪"用**软件轮询 + 时间门限**实现，
  详见 §3.2 与 `docs/TIMING.md` §4。

### 1.4 模拟前端（8 通道板实际接法，见 `docs/SCHEMATIC_CHANGES.md`）

* **没有串联电阻**：原设计在每路输入串 6.7kΩ 已删除（BCS 恒流源的压降会变成
  不可接受的失调），每通道用 **2 × 1 µF 对地 + 1 × 100 nF 差分电容**滤波，
  配合固件软件滤波。
* `AINP`/`AINN` 各接偏置电阻到 `AVDD/2`（开路检测的手段）。
* AVDD/DVDD 从 LDO 出来后经磁珠隔离，0.1 µF 就近去耦到同一地平面上。
* **增益**：当前按需求用 `GAIN = 1` + `PGA` 旁路（共模范围可到 `AVSS−0.1V ~ AVDD+0.1V`，
  适合热电偶接地的情况）。K 型在 1250 °C 时只有 50.6 mV，
  对着 ±2.048 V 的满量程只用了 1.2%，分辨率约 0.244 µV/LSB ≈ 0.006 °C/LSB（理论值）。
  想提高有效分辨率，把 `board_config.h` 里的 `ADS1220_GAIN_1` 改成 `ADS1220_GAIN_32`
  并同时去掉 PGA 旁路（此时必须保证偏置电阻把共模抬到中间电位），
  然后**重做一次失调校准**（增益变了，失调也变）。

---

## 2. 文件清单

```
Core/Inc/
├── board_config.h        ★ 单一事实来源: 引脚映射 / 片数 / 全部时序常量 / 应用配置
├── ads1220.h             ★ 寄存器位域、指令、速率表、驱动 API
├── spi_driver.h          ★ SPI1 总线层 API (片选/事务/每片 DRDY 轮询/延时)
├── thermocouple.h        ★ NIST K 型多项式 + 冷端补偿 API
├── uart_protocol.h       ★ 协议帧定义、上行状态帧布局、事件类型
├── main.h                固件版本
├── gpio.h / spi.h / usart.h / stm32f1xx_it.h / stm32f1xx_hal_conf.h
Core/Src/
├── main.c                ★ 非阻塞轮询状态机 / 命令分发 / 上报
├── spi_driver.c          ★ SPI1 事务 + CS 时序 + 每片 DRDY 轮询初始化 + DWT 微秒延时
├── ads1220.c             ★ 24bit 读取 / WREG·RREG / 回读校验 / 内部短路失调校准
├── thermocouple.c        ★ ITS-90 正逆多项式 / 冷端补偿 / 断线判定 / 自检
├── uart_protocol.c       ★ CRC-16/MODBUS / 收帧状态机 / 中断发送环形缓冲
├── gpio.c / spi.c / usart.c / stm32f1xx_it.c / stm32f1xx_hal_msp.c
├── system_stm32f1xx.c
docs/
├── TIMING.md             ★ 所有时序细节 + 需求与数据手册的逐条对照 + 时间预算
└── BUILD.md              ★ Keil / Makefile 构建、CubeMX 重新生成须知、F407 移植
pc_ui/
├── __init__.py
├── protocol.py           协议层 (固件 uart_protocol.c 的 Python 对偶实现, 自带自检)
└── serial_reader.py      串口后台线程 (独占串口 / 断线自动重连 / 收帧进 queue)
host_parser.py            命令行工具 (复用 pc_ui.protocol, 不重复协议逻辑)
Makefile / STM32F103xB_FLASH.ld
MDK-ARM/Thermocouple.uvprojx   Keil 工程 (新增 4 个 .c 已加入编译)
```

> 协议实现**只有一处**：`pc_ui/protocol.py`。
> `host_parser.py`（命令行）和 `pc_ui/serial_reader.py`（GUI 后台线程）都基于它，
> 避免两份 CRC/组帧代码各自漂移。

---

## 3. 固件架构

### 3.1 分层

```
main.c           应用状态机 (轮询 / 命令 / 上报)
  ├── ads1220.c         ADS1220 寄存器语义层
  │     └── spi_driver.c   SPI1 事务 + CS 时序 + 每片 DRDY 轮询
  ├── thermocouple.c    NIST 多项式 + 冷端补偿
  └── uart_protocol.c   CRC + 组帧/拆帧 + 中断收发
        └── usart.c / stm32f1xx_it.c
```

### 3.2 采集：为什么要"广播 START + 等全部 DRDY 就绪"

每片一根 DRDY（PA0~PA3），固件用**软件轮询**判断"全部就绪"：

```
每切换一次 MUX / TS：
  对 4 片逐片发 WREG(改寄存器) + START/SYNC(0x08)
      ↑ START/SYNC 会复位数字滤波器并重启转换 → 4 片时间上重新对齐，
        之后 4 根 DRDY 会在几乎同一时刻拉低
  然后等：  (已经过 ≥ 0.9 个转换周期)  AND  (4 根 DRDY 全部为低)
  超时 (转换周期 + 25 ms) 就记一次故障并继续 —— 流程永不卡死
```

一轮的四个阶段：

| 阶段 | 动作 | 读回 |
|---|---|---|
| 1 | MUX = `AIN0/AIN1` | `s_codeA[4]` |
| 2 | MUX = `AIN2/AIN3` | `s_codeB[4]` |
| 3 | `CONFIG1.TS = 1` | `s_codeT[4]`（内部温度 = 冷端） |
| 4 | 恢复 `TS=0`、MUX=A，重新 START → **同时**做冷端补偿、组帧、上报 | — |

20 SPS 时每阶段约 50 ms → 整轮约 150 ms，8 路同时刷新（约 6.7 Hz）。

需求里的 **"每完成 N 次转换切换一次 MUX"** 由
`board_config.h: TC_MUX_HOLD_CONVERSIONS` 控制，默认 `1`（每次转换后切换）。
改成 `N>1` 时每个通道会连读 N 次再切换（`N=2` 时整轮约 200 ms）。

### 3.3 非阻塞保证

| 位置 | 做法 |
|---|---|
| 主循环 | 每阶段轮询 4 根 DRDY 电平 + 时间门限判定"全部就绪"，**绝不做阻塞等待** |
| `USART1_IRQHandler` | 逐字节喂协议状态机（F1 的 UART 只有 1 字节接收寄存器，必须在 ISR 里取走）；发送完成时搬运环形缓冲的下一段 |
| 主循环 | 只做"查就绪 → 读 SPI → 算 → 入队发送"，**没有任何等待转换完成的 `while` 阻塞** |
| 串口发送 | 512 字节环形缓冲 + `HAL_UART_Transmit_IT`，一帧 133 字节只是内存拷贝 |
| 唯一的阻塞 | 上电时的失调校准（4 × 50 ms ≈ 210 ms），可以用 `TC_OFFSET_CAL_ENABLE 0` 关掉 |

---

## 4. 通信协议

### 4.1 帧格式

```
PC  -> STM32 :  AA  CMD  LEN  DATA[0..LEN-1]  CRC_LO  CRC_HI
STM32 -> PC  :  55  CMD  LEN  DATA[0..LEN-1]  CRC_LO  CRC_HI
```
* `LEN` = DATA 段字节数
* `CRC` = **CRC-16/MODBUS**（poly `0xA001` 反射，init `0xFFFF`），覆盖
  **帧头 → 最后一个 DATA 字节**，**低字节在前**。
  标准向量：`b"123456789"` → `0x4B37`

### 4.2 上行

| CMD | 名称 | DATA |
|---|---|---|
| `0x10` | 温度上报 | **32 × float32 小端**（°C），顺序 ch0…ch31。**协议固定 32 槽**（本板用前 8 槽，其余 = **NaN** 位型 `0x7FC00000`）。仅 `TC_UPLINK_MODE=0/2` 时发送 |
| `0x11` | 状态上报 | 24 字节，见下表 |
| `0x12` | ★**原始上报** | **192 字节** = 32 × float32 热电势（**µV**，已减失调）+ 16 × float32 冷端温度（**°C**，每片一个）。固件**不算温度**，由上位机查分度表换算。默认模式（`TC_UPLINK_MODE=1`） |

> ★ **温度由谁换算**（`board_config.h` 的 `TC_UPLINK_MODE`，默认 `1`）：
> 固件只把 ADS1220 的 24 bit 码值变成热电势 µV 和冷端 °C，NIST 分度表 / 插值 /
> 单支标定全部交给上位机（`pc_ui/tc_table.py`）。这样换分度号（K/J/T/E/N）、
> 换厂家粗表都不用重编译固件，CSV 里同时存了 µV 与 °C 还能**离线重算**。
>
> 精度对比（K 型，冷端 25 °C，全量程最大误差）：固件 float32 多项式 **0.054 °C**
> vs 上位机 pchip 查表 **0.016 °C** —— 两者都远低于 ADS1220 的噪声底
> （约 0.05–0.07 °C），所以换到上位机图的是**灵活性**，不是精度。
>
> 状态帧 `flags` 的 **`0x40`**（`ST_FLAG_RAW_UPLINK`）会置位，上位机据此知道
> "固件发的是原始帧，温度要自己算"。详见 `docs/TC_TABLE_FORMAT.md` 第 8 节。

原始帧 DATA 布局（偏移从 0 开始）：

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 128 | 32 × float32 热电势（µV），无效 = NaN |
| 128 | 64 | 16 × float32 冷端温度（°C，每片一个），无芯片 = NaN |

> 冷端**必须**一起发：冷端补偿 `V_total = V_TC + E(T_CJ)` 只能由同一个算法完成，
> 只给 µV 不给冷端温度，上位机算不出热端温度。
> 码值 → µV 这一步是无损的（ADS1220 增益是 2 的幂，且 `2048000/8388608 = 0.244140625`
> 是二进制精确值，float32 尾数 24 位 = 码值位数）。

状态帧 DATA 布局（偏移从 0 开始）：

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 1 | `run_state` 0=停止 / 1=运行 |
| 1 | 1 | 当前 `DR[2:0]`（已移位值：`0x00`=20SPS … `0xC0`=1000SPS） |
| 2 | 1 | 当前 `50/60[1:0]`（`0x00`/`0x10`/`0x20`/`0x30`） |
| 3 | 2 | `chip_ok_mask` LE，bit n=1 表示第 n 片上电回读校验通过 |
| 5 | 2 | `chip_err_mask` LE，bit n=1 表示第 n 片本轮 SPI 出错 |
| 7 | 4 | `round_count` LE，已完成测量轮次 |
| 11 | 4 | `uptime_ms` LE，上电至今 ms |
| 15 | 2 | `drdy_irq_count` LE（低 16 位），"全部就绪"次数（老 16 片版叫"合并 DRDY 中断次数"） |
| 17 | 2 | `spi_err_count` LE，SPI 事务失败累计 |
| 19 | 2 | `open_tc_count` LE，断线累计次数 |
| 21 | 1 | `flags`，见下 |
| 22 | 2 | `phase_timeout_count` LE，等待 DRDY 超时次数 |

`flags` 位：`0x01` 自检失败 / `0x02` DRDY 静默（>1 s 无"全部就绪"）/
`0x04` 回读校验不通过 / `0x08` 串口发送缓冲溢出丢帧 / `0x10` 收到过 CRC 错的下行帧 /
`0x20` 有片一直不就绪（看 `chip_err` 位图定位）/
`0x40` ★固件处于**原始上报**模式（发 `0x12`，温度需上位机自己算）。

### 4.3 下行

| CMD | 名称 | DATA | 应答 |
|---|---|---|---|
| `0x01` | 设置采样率 | `LEN=2`: `[DR索引 0..6, 抑制索引 0..3]`；`LEN=1`: `[(DR<<4)\|抑制]`。也接受"已移位"写法（`0x20`/`0x40`/…/`0xC0`）。DR 索引 `0`=20SPS `1`=45 `2`=90 `3`=175 `4`=330 `5`=600 `6`=1000；抑制索引 `0`=不抑制 `1`=同时抑制50+60 `2`=仅50 `3`=仅60 | `0x11` 状态帧 |
| `0x02` | 启动/停止 | `[0 停止 / 1 启动]` | `0x11` 状态帧 |
| `0x03` | 读取单次温度 | 省略或 `[0xFF]` = 全部通道；`[0..31]`（协议槽位，本板 0..7 有效）= 只报该通道（其余填 NaN） | `0x10` 温度帧 |

> ⚠️ 50/60Hz 抑制**只能**配合 20 SPS 正常模式（手册 Table 8-13）。
> 如果下发了别的 DR + 抑制组合，固件会强制把抑制写 0，
> 并在随后回的状态帧里给出**实际生效**的值。

### 4.4 实帧示例（SEQ 无、本板 8 路全 25.00 °C，其余槽 NaN）

```
上行温度帧 LEN = 32*4 = 128 (0x80) → 总长 133 字节
55 10 80  00 00 C8 41  00 00 C8 41 ... (共 128 字节) ...  CRC_LO CRC_HI
```
上位机 `python host_parser.py --test` 会用同一套 CRC/组帧/拆帧逻辑做自检。

---

## 5. ADS1220 寄存器配置（最终值）

```
WREG 0x43 (从 0x00 起写 4 个字节):
  CONFIG0 = 0x01   MUX=0000b(AIN0/AIN1), GAIN=000b(×1), PGA_BYPASS=1
  CONFIG1 = 0x05   DR=000b(20SPS), MODE=00b(正常), CM=1(连续), TS=0, BCS=1
  CONFIG2 = 0x10   VREF=00b(内部2.048V), 50/60=01b(同时抑制50+60Hz), PSW=0, IDAC=000b
  CONFIG3 = 0x00   I1MUX/I2MUX=0(无IDAC), DRDYM=0(只用专用 DRDY 引脚)
```

运行中动态改写：

```
CH_A : CONFIG0 = 0x01      (MUX = 0000b)
CH_B : CONFIG0 = 0x51      (MUX = 0101b)
冷端 : CONFIG1 = 0x07      (TS = 1; 此时 CONFIG0 完全无效, 强制用内部基准)
还原 : CONFIG1 = 0x05, CONFIG0 = 0x01, 再 START/SYNC
```

**上电流程**（`ADS1220_PowerUpInit()`）：

```
等待 ≥ 50 ms  (电源 + 上电复位)
逐片 RESET (0x06)                       → 等 1 ms (手册要求 ≥ 50 µs + 32×t_CLK = 57.8 µs)
逐片 WREG 0x43 + 4 字节
逐片 RREG 0x23 回读校验 → chip_ok_mask
逐片 START/SYNC (0x08) 启动连续转换
```

> 📌 **需求里的 "SELF CAL 指令 0x04" 不存在。** ADS1220 只有 6 条指令
> （RESET / START-SYNC / POWERDOWN / RDATA / RREG / WREG）。
> 固件按手册 9.1.5 实现了**内部短路失调校准**：
> 把 MUX 置 `1110b`（AINP/AINN 短接到 mid-supply）→ 采 4 次平均 → 存起来，
> 之后每次读数减掉。详见 `docs/TIMING.md` §7。
>
> 其余几处需求与数据手册的位位置差异（FIR 在 CONFIG2 而不是 CONFIG1、
> "同时抑制 50/60Hz"是 `01b` 而不是 `11b`、DR 在 CONFIG1、连续转换是 CM 位……）
> **都在 `docs/TIMING.md` §10 逐条列出并说明了固件的处理方式**，
> 功能意图 100% 保留。

---

## 6. 冷端补偿算法

严格按手册 9.2.1.2 的 4 步：

```
1. 测热电偶电压        V_TC  [µV]   (差分通道, 1 LSB = 0.244140625 µV @Gain=1)
2. 测冷端温度          T_CJ  [°C]   (CONFIG1.TS=1, 内部温度传感器, ±0.5°C)
3. 冷端等效热电势      E_CJ = K型正多项式(T_CJ)          [µV]
4. 求和再反解热端      T_hot = K型逆多项式(V_TC + E_CJ)  [°C]
```

多项式系数取自 **NIST Monograph 175 / ITS-90**：

* 正多项式分 `-270~0 °C`（10 阶）与 `0~1372 °C`（9 阶 + 指数修正项）
* 逆多项式分 `-200~0 °C` / `0~500 °C` / `500~1372 °C` 三段

实现要点：全部用 **float + 秦九韶(Horner)**。
F103 没有 FPU，单精度比 double 快好几倍；Horner 展开还把中间量控制在
和结果同量级，避开了 NIST 逆多项式在大 E 处的灾难性抵消。
实测算术链路误差 **< 0.03 °C**（逆多项式自身在段边界有约 0.05 °C 的拟合误差）。

上电跑 `TC_SelfTest()`，用 `-100/0/25/500/1000/1372 °C` 六个标准点
（对应 `-3553.63 / 0 / 1000.24 / 20644.29 / 41275.61 / 54886.36 µV`）
做 `t → E → t` 回环 + 冷端补偿闭环验证。

---

## 7. 错误处理一览

| 故障 | 检测方式 | 表现 |
|---|---|---|
| SPI 超时 / HAL 错误 | `HAL_SPI_TransmitReceive` 返回非 `HAL_OK` | 计数 + 自动 `HAL_SPI_Abort` 恢复总线；该片本轮数据作废，**不影响其他片** |
| 芯片无响应 / 没焊 | 上电 `RREG` 回读与写入值逐字节比对（MISO 有下拉，坏片读回全 0） | `chip_ok_mask` 对应位为 0；该片两个通道上报 NaN |
| 断线（开路） | ① 码值到 ±满量程 `0x7FFFFF`/`0x800000`；② 热电势超出 K 型物理量程 `[−12, +60] mV` | 该通道上报 NaN，`open_tc_count++` |
| 冷端温度超范围 | 内部温度传感器换算结果不在 `[−40, +125] °C` | 整片两路都判无效 |
| 全就绪一直不来 | 每个等待窗口超时（转换周期 + 25 ms） | `phase_timeout_count++`，流程继续 |
| 所有片 DRDY 静默 | 主循环 1 s 内"全部就绪"没出现 | `flags` 置 `DRDY_SILENT(0x02)` |
| 个别片 DRDY 一直拉高 | 等待窗口里逐片轮询发现某片一直不就绪 | `flags` 置 `DRDY_PARTIAL(0x20)`，`chip_err` 对应位置位 |
| 热电偶多项式自检失败 | 上电 `TC_SelfTest()` | `flags` 置 `SELFTEST(0x01)` |
| 串口发送跟不上 | 环形缓冲满 | 整帧丢弃 + `TX_OVERFLOW(0x08)` |
| 下行帧 CRC 错 | 收帧状态机校验 | 丢帧 + `CRC_ERR(0x10)` |

---

## 8. 快速上手

```bash
# 1) 编译（Keil 双击 MDK-ARM/Thermocouple.uvprojx 按 F7；或无 Keil 环境用）
make -j8

# 2) 上位机协议自检（不需要硬件）
python host_parser.py --test          # 或: python -m pc_ui.protocol

# 3) 连上硬件看数据
python host_parser.py COM3
```

详细说明见：

* **`docs/TIMING.md`** —— 全部时序常量、t_DATA/t_RESET/转换时间/DRDY 行为、
  需求与数据手册的逐条对照、串口时间预算
* **`docs/BUILD.md`** —— Keil/Makefile 构建、CubeMX 重新生成后必须检查的 4 件事、
  换成 STM32F407 要改哪些地方、出问题时的排查顺序
* **`PLAN.md`** —— 相对原 MAX31855 模板的逐文件改动清单
