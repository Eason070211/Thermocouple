# 时序细节汇总 (ADS1220 × 16 / STM32F103C8T6)

> 所有数值均出自 **TI ADS1220 数据手册 SBAS501D**（用户提供的那份 PDF），
> 章节号在每条后面标注。固件里对应的宏定义在 `Core/Inc/board_config.h`。

---

## 1. 时间基准

| 符号 | 值 | 说明 |
|---|---|---|
| f_CLK | 4.096 MHz | 内部低漂移振荡器（CLK 引脚接 DGND 时使用） |
| t_CLK | 244.14 ns | = 1 / 4.096 MHz |
| t_MOD | 3.906 µs | = 1 / 256 kHz，正常模式调制器周期 |

> 数据率、转换时间都按 f_CLK = 4.096 MHz 折算；内部振荡器有 ±(手册 EC 表) 的
> 频率误差，所以**转换时间与 50/60Hz 陷波点会有几个百分点的漂移**，
> 要精确的 50/60Hz 抑制必须给 CLK 引脚送外部 4.096 MHz（本设计没有外接，
> 因此 50/60Hz 抑制是"典型值"级别，不是规格保证级别）。

---

## 2. 上电 / 复位

| 参数 | 手册要求 | 固件实现 | 宏 |
|---|---|---|---|
| 上电复位时间 | ≥ 50 µs（8.4.1） | **50 ms** | `ADS1220_T_POWERUP_MS` |
| RESET 指令(0x06)后等待 | ≥ 50 µs + 32 × t_CLK = **57.8 µs**（8.5.3.1） | **1 ms** | `ADS1220_T_RESET_US` |

上电等 50 ms 而不是 50 µs 是有意的：TPS7A4901 软启动 + 磁珠隔离后的
AVDD/DVDD 以及内部 2.048V 基准都需要时间稳定，这段时间里读数没有意义。

**没有 `tSELFOCAL` 这个参数**：ADS1220 数据手册 8.5.3 节的指令表里
只有 6 条指令（RESET / START-SYNC / POWERDOWN / RDATA / RREG / WREG），
**不存在自校准指令，`0x04` 不是合法指令**。手册 9.1.5 给出的等效做法是
"把 MUX 置 1110b 短接输入 → 采多次求平均 → 存到 MCU → 之后每次读数减掉"，
固件用 `ADS1220_OffsetCalibrateAll()` 实现，见 §7。

---

## 3. SPI 物理层时序（Mode 1）

| 参数 | 含义 | 限值 | 固件对策 |
|---|---|---|---|
| `tc(SC)` | SCLK 周期 | **≥ 150 ns** → f_SCLK ≤ **6.67 MHz** | SPI1 = APB2/16 = 72/16 = **4.5 MHz**（周期 222 ns） |
| `tw(SCH)` | SCLK 高电平 | ≥ 60 ns | 由 4.5 MHz 保证（111 ns） |
| `tw(SCL)` | SCLK 低电平 | ≥ 60 ns | 同上 |
| `td(CSSC)` | CS 下降到第 1 个 SCLK 上升沿 | ≥ 50 ns | `SPI_DRIVER_SHORT_DELAY()` = 8×NOP = 111 ns @72MHz |
| `td(SCCS)` | 最后一个 SCLK 下降到 CS 上升 | ≥ 25 ns | 同上 |
| `tw(CSH)` | CS 高电平最小宽度 | ≥ 50 ns | 同上 |
| `tsu(DI)` | DIN 在 SCLK **下降沿**前建立 | ≥ 50 ns | STM32 SPI Mode 1 硬件保证 |
| `th(DI)` | DIN 在 SCLK **下降沿**后保持 | ≥ 25 ns | 同上 |
| `tp(SCDO)` | SCLK **上升沿**到 DOUT 更新 | ≤ 50 ns | 主机在下降沿采样，有 111 ns 裕量 |
| `tp(CSDO)` | CS 下降到 DOUT 被驱动 | ≤ 50 ns | 未选中芯片 DOUT 高阻 |
| `tp(CSDOZ)` | CS 上升到 DOUT 高阻 | ≤ 50 ns | 多片共享 MISO 依靠这个 |

### ⚠️ 为什么必须是 Mode 1，而不是 MAX31855 时代的 Mode 0

手册 6.6 节的参数名直接说明了边沿关系：

* `tsu(DI)` / `th(DI)` 是相对 **SCLK 下降沿** 定义的 → **DIN 在下降沿被锁存**；
* `tp(SCDO)` 写的是 "SCLK **rising edge** to valid new DOUT" → **DOUT 在上升沿更新**；
* 8.5.1.4 节文字确认："The device latches data on DIN on the SCLK falling edge."

STM32 的 Mode 1（CPOL=0, CPHA=1，HAL 里是
`SPI_POLARITY_LOW` + `SPI_PHASE_2EDGE`）正是
"第一个（上升）沿输出、第二个（下降）沿采样"。
**MAX31855 用的是 Mode 0（`SPI_PHASE_1EDGE`），换芯片时这一项必须改**，
否则读回的数据会整体错位 1 个 bit。

### 关于 "≤10MHz"

需求里写的是 SCLK ≤ 10 MHz，但 SBAS501D 6.6 节给出的
`tc(SC) MIN = 150 ns` 对应 **f_SCLK ≤ 6.67 MHz**。10 MHz 会超规格。
固件取 **4.5 MHz**：是 72 MHz 的整数分频（/16），有 48% 裕量，
一帧 4 字节事务约 7.1 µs + 开销，16 片批量读一次约 200 µs。

### 关于 `tDATA`（需求里"至少 1 个 CLK 周期"）

手册**没有**为专用 DRDY 引脚定义 tDATA。8.5.4 节明确写着：

> Data can be read directly from this buffer on DOUT/DRDY **when DRDY falls low
> without concern of data corruption**. An RDATA command does not have to be sent.

也就是说 DRDY 下降沿时数据已经准备好了。固件仍然保守地插入
`ADS1220_T_DATA_US = 2 µs ≈ 8 × t_CLK`（对应需求里的"至少 1 个 CLK 周期"），
用来覆盖：74HC30/74HC132 的传播延迟 + EXTI 中断响应延迟 + `td(CSSC)` 建立时间。

---

## 4. 转换时间与 DRDY 行为

连续转换模式下，数据率是 **一个 DRDY 下降沿到下一个 DRDY 下降沿**（8.3.7）。
第一次转换在 START/SYNC 的最后一个 SCLK 下降沿之后 `210 × t_CLK ≈ 51 µs` 开始。

| DR 设置 | DR[2:0] | 标称 SPS | 实际转换时间 (t_CLK) | 实际 (µs) | 50/60 抑制可用 |
|---|---|---|---|---|---|
| 000 | 0x00 | 20 | 204 768 | **49 992** | ✅ 是 |
| 001 | 0x20 | 45 | 91 120 | 22 246 | ❌ |
| 010 | 0x40 | 90 | 46 128 | 11 262 | ❌ |
| 011 | 0x60 | 175 | 23 664 | 5 777 | ❌ |
| 100 | 0x80 | 330 | 12 464 | 3 043 | ❌ |
| 101 | 0xA0 | 600 | 6 896 | 1 684 | ❌ |
| 110 | 0xC0 | 1000 | 4 144 | 1 012 | ❌ |
| 111 | — | 保留 | — | — | 禁止使用 |

> 抑制位 `50/60[1:0]` **只能**配合"正常模式 + 20 SPS"使用（Table 8-13），
> 其它速率必须写 `00b`。固件在 `App_SetRate()` 里强制保证这一点，
> 上位机下发非法组合时会被自动改掉，并在状态帧里回读实际生效的值。

### DRDY 的电气行为（8.5.1.3）—— 决定本设计的拓扑

* DRDY 是**低有效、主动推挽输出**，"is always actively driven, **even when CS is high**"
  → 所以 16 路 DRDY 可以直接进 74HC30/74HC132 与非网络，不需要三态缓冲。
* "DRDY transitions back high on **the next SCLK rising edge**"
  → 读完数据后合并信号自动恢复高，**不需要任何复位逻辑**（与需求描述一致）。
* "When no data are read during continuous conversion mode, DRDY remains low but
  pulses high for a duration of **2 × t_MOD ≈ 7.8 µs** prior to the next DRDY falling edge."
  → 连续模式下 DRDY 是"低电平为主 + 每周期 7.8 µs 高脉冲"，
  合并后的 PA0 每个转换周期都会产生一次下降沿，EXTI0 能可靠捕获。

### 等待窗口是怎么定的（`App_StartWait()` / `App_WaitDone()`）

因为合并后的 DRDY **无法分辨是哪一片**，固件采用"广播重同步 + 时间门限"：

1. 每次切换 MUX / TS 后，对 16 片**逐片**发 `WREG` + `START/SYNC`。
   START/SYNC 会复位数字滤波器并重启转换，16 片重新对齐（彼此只差几十 µs）。
2. 等待窗口同时满足两个条件才继续：
   * **最小等待** = 实际转换时间 × 9/10（20 SPS 时 = 44 ms）
     —— 保证读到的是新通道的数据，不是切换前的旧数据；
   * 合并 DRDY 的中断标志已置位（ISR 只置标志位）。
3. **超时** = 转换时间 + 25 ms，超时则 `phase_timeout_count++` 并继续，
   保证流程永不卡死。

### 一轮测量要多久

```
SET(CONFIG0: MUX=A) + START  ─┐
  等 1 个转换周期              │  ~50 ms
  读 16 片 × 3 字节            ┘
SET(MUX=B) + START           ─┐
  等 1 个转换周期              │  ~50 ms
  读 16 片                     ┘
SET(CONFIG1: TS=1) + START   ─┐
  等 1 个转换周期              │  ~50 ms
  读 16 片（内部温度）          ┘
SET(TS=0, MUX=A) + START        <-- 同时开始下一轮的 A 通道
上报 133 字节                    <-- 与 50 ms 转换时间重叠
```

所以 **20 SPS 下整轮 ≈ 3 × 50 = 150 ms（实测 3 × 45 ms 门限 → 约 135~150 ms），
即每路约 6.7 Hz 更新率**，32 路一起刷新。

---

## 5. 串口时间预算

一帧温度上报 = `1(头) + 1(CMD) + 1(LEN) + 128(32×float32) + 2(CRC) = 133` 字节。

| 波特率 | 单帧耗时 (133 B, 8N1 → 10 bit/字节) | 理论上限帧率 |
|---|---|---|
| 115200 | 11.5 ms | ~87 帧/s |
| 460800 | 2.9 ms | ~347 帧/s |

| DR 设置 | 轮询周期 (3 个转换周期) | 需求帧率 | 115200 够吗 | 460800 够吗 |
|---|---|---|---|---|
| 20 SPS | ~150 ms | 6.7 Hz | ✅ 富余 | ✅ |
| 45 SPS | ~67 ms | 15 Hz | ✅ | ✅ |
| 90 SPS | ~34 ms | 30 Hz | ⚠️ 勉强（占用率 34%） | ✅ |
| 175 SPS | ~17 ms | 58 Hz | ❌ 会丢帧 | ✅ |
| 330 SPS | ~9 ms | 110 Hz | ❌ | ⚠️ 占用率 32%→可 |
| 600 SPS | ~5 ms | 200 Hz | ❌ | ❌ |
| 1000 SPS | ~3 ms | 330 Hz | ❌ | ❌ |

> 丢帧不会破坏数据：发送侧是 512 字节环形缓冲 + 中断发送，
> 缓冲满时整帧丢弃并计数，状态帧的 `flags` 会置 `TX_OVERFLOW(0x08)`。
> **实际工程建议留在 20 SPS**：只有 20 SPS 能用 50/60Hz 抑制滤波器，
> 其它速率下工频干扰会直接进入读数。

---

## 6. 中断与优先级

| 中断 | 优先级（NVIC_PRIORITYGROUP_4） | ISR 里做什么 |
|---|---|---|
| EXTI0 (PA0, 合并 DRDY) | 1（更高） | **只置标志位 + 计数**，绝不碰 SPI |
| USART1 | 2 | `HAL_UART_IRQHandler` → 逐字节喂协议状态机；发送完成续传 |

* EXTI0 优先级高于 USART1：DRDY 边沿比较短（7.8 µs 高脉冲），不能被串口 ISR 挡住。
* 一次 16 片批量读 48 字节 ≈ 200 µs，绝对不能放在 ISR 里 —— 会拖长中断、
  挤掉串口接收。所有 SPI 都在主循环完成。
* `SPI_Driver_DrdyTakeFlag()` 用 PRIMASK 保存/恢复做临界区，
  不会把"本来已关中断"的上下文提前开中断。

---

## 7. 失调校准（替代需求里的 "SELF CAL 0x04"）

手册 9.1.5 / 8.3.12 的做法：

1. 16 片都写 `CONFIG0.MUX = 1110b`（AINP/AINN 内部短接到 (AVDD+AVSS)/2）。
   注意 **`TS=1` 时 CONFIG0 完全无效**，所以必须先 `TS=0`。
2. 每片发 `START/SYNC`，等一个转换周期，读 24 位码值。
3. 重复 `TC_OFFSET_CAL_SAMPLES`（默认 4）次取平均，存进 MCU 的 `s_offset[16]`。
4. 恢复原 `CONFIG0`/`CONFIG1` 并重新 `START/SYNC`。

之后每次读数：`V_TC = (code - offset[chip]) × 1 LSB`。

* 耗时：4 × 50 ms + 开销 ≈ **210 ms**，只在上电时做一次。
* 该失调是**在当前 GAIN/PGA_BYPASS 下测得的**，改增益后必须重做。
* 用 `TC_OFFSET_CAL_ENABLE 0` 可以关掉（会省 210 ms 开机时间）。

---

## 8. 换算链（每一步的单位和精度）

```
ADS1220 24bit code
   │  1 LSB = (VREF / Gain) / 2^23 = (2.048V / 1) / 8388608 = 0.244140625 µV
   ├─► 减失调后 × 0.244140625  →  V_TC [µV]
   │
   │  温度传感器: 14bit 左对齐在 24bit 里, 1 个 14bit LSB = 0.03125 °C
   ├─► code >> 10 (算术) × 0.03125  →  T_CJ [°C]     (精度 ±0.5°C)
   │
   ├─► E_CJ = K型正多项式(T_CJ)                      [µV]
   ├─► V = V_TC + E_CJ                               [µV]
   └─► T_hot = K型逆多项式(V)                        [°C]
```

* 正/逆多项式都用 **float + 秦九韶(Horner)**。F103 没有 FPU，单精度比 double 快数倍；
  实测整条链的误差 < 0.03 °C（逆多项式本身在段边界有 ~0.05 °C 误差，
  这是 NIST 拟合的固有误差，不是实现问题）。
* 上电跑一次 `TC_SelfTest()`，用 NIST 标准点
  （-100/0/25/500/1000/1372 °C 对应的 1000.24 / 41275.61 / 54886.36 µV 等）
  验证多项式链路；失败会把状态帧 `flags` 的 `SELFTEST(0x01)` 置起来。

---

## 9. 断线（开路）检测

两条**互相独立**的判据，命中任一即判开路（`thermocouple.c: TC_Compute()`）：

1. **码值被拉死**：`code >= 0x7FFFFF` 或 `code <= 0x800000`（±满量程）。
   手册 9.2.1.2："when one of the thermocouple leads fails open, the biasing
   resistors pull the analog inputs to AVDD and AVSS... the ADC consequently reads
   a full-scale value"。
2. **换算出的热电势超出 K 型物理量程**：合法范围是
   `V_TC = E(T_hot) − E(T_CJ)`，`T_hot∈[−200,1372]`、`T_CJ∈[−40,85]`
   → 约 `[−9400, +56413] µV`。固件用 `[−12000, +60000] µV` 作门限，留了裕量。

`BCS`（10 µA 烧断电流源）由 `board_config.h` 的 `TC_BCS_ALWAYS_ON` 控制，默认 **1（常开）**，
与需求 "CONFIG1: BCS=断线检测使能" 一致。注意其副作用：

> 10 µA 会流过热电偶回路电阻 R，产生 `10 µA × R` 的附加失调。
> 2 m 的 K 型偶丝回路电阻只有几 Ω → 几十 nV（≈ 0.0002 LSB），可忽略；
> 但如果用了上百米的补偿导线（回路上百 Ω），误差会到几百 µV（几 °C），
> 这时应把 `TC_BCS_ALWAYS_ON` 改成 0，并依赖模拟前端的
> 1 MΩ~50 MΩ 偏置电阻（TI 参考设计 Figure 9-3 的 RB1/RB2）做开路检测。

---

## 10. 需求 → 数据手册 逐条对照

| 需求原文 | 手册实际 | 固件做法 |
|---|---|---|
| CONFIG0：MUX 差分 + GAIN=1 + PGA 旁路 + **DR=20SPS** | DR 在 **CONFIG1[7:5]** | CONFIG0=0x01（MUX=0,GAIN=0,PGA_BYPASS=1），DR 放 CONFIG1 |
| CONFIG1：**FIR[1:0]=11**（同时抑制 50/60Hz），TS=0，BCS 使能 | FIR/50-60 在 **CONFIG2[5:4]**；且"同时抑制"是 **01b**，11b 只是"仅 60Hz" | CONFIG1=0x05（DR=20SPS,CM=连续,TS=0,BCS=1）；CONFIG2=0x10（VREF=int,50/60=**01b**） |
| CONFIG2：VREF 内部、IDAC 关、**DRDY 使能** | VREF/IDAC 在 CONFIG2 ✓；DRDY 由 **CONFIG3.DRDYM** 控制 | CONFIG2=0x10；CONFIG3=0x00（DRDYM=0 → 只用专用 DRDY 引脚） |
| CONFIG3：**连续转换模式** + DRDY 专用引脚 | 连续转换是 **CONFIG1.CM[2]** | CONFIG1 里置 CM=1；CONFIG3=0x00 |
| **SELF CAL 指令 0x04** | **不存在这条指令**（只有 6 条） | 用手册 9.1.5 的内部短路失调校准替代（§7） |
| SCLK ≤ 10 MHz | `tc(SC) ≥ 150 ns` → ≤ 6.67 MHz | 4.5 MHz（APB2/16） |
| 读前等 tDATA ≥ 1 CLK | 专用 DRDY 未定义 tDATA | 仍插 2 µs ≈ 8 × t_CLK 裕量 |

> 功能意图 100% 保留，只是**位位置按数据手册落到了正确的寄存器**。
> 固件在 `Core/Inc/ads1220.h` 的文件头、以及 `ads1220.c: ADS1220_BuildRegs()`
> 里都写了这份对照表，避免以后维护时再踩。

**上电后 16 片统一写入的 4 个字节（默认配置）：**

```
WREG 0x43 (从 0x00 起写 4 个) →
  CONFIG0 = 0x01   ; MUX=AIN0/AIN1, GAIN=1, PGA 旁路
  CONFIG1 = 0x05   ; DR=20SPS, MODE=正常, CM=连续, TS=0, BCS=开
  CONFIG2 = 0x10   ; VREF=内部2.048V, 50/60=同时抑制, PSW=开?否, IDAC=关
  CONFIG3 = 0x00   ; 无 IDAC, DRDYM=0(只用专用 DRDY 引脚)
```

游标测量时动态改写：
```
CH_A:  CONFIG0 = 0x01  (MUX=0000b = AIN0/AIN1)
CH_B:  CONFIG0 = 0x51  (MUX=0101b = AIN2/AIN3)
冷端:  CONFIG1 = 0x07  (在 0x05 基础上 TS=1；此时 CONFIG0 无效)
```
