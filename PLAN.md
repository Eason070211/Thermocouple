# 16 × ADS1220 / 32 路热电偶测温系统 —— 改造实施记录

> 从原模板（**STM32F103C8 + 4 路 MAX31855**）改造为
> **STM32F103C8 + 16 片 ADS1220 / 32 路 K 型热电偶**。
> 本文是"相对模板改了什么、为什么改"的清单；使用方法见 `ReadMe.md`，
> 时序细节见 `docs/TIMING.md`，构建见 `docs/BUILD.md`。

---

## 0. 硬件拓扑（已确认）

| 项 | MAX31855 模板 | 本工程 |
|---|---|---|
| 传感器 | MAX31855 × 4（每片 1 路 + 自带冷端） | ADS1220 × 16（每片 2 路差分 + 内部温度传感器做冷端） |
| 通道数 | 4 | **32** |
| SPI1 | **RX-Only 单工**，Mode 0，562.5 kHz | **全双工（新增 MOSI=PA7）**，**Mode 1**，4.5 MHz |
| 片选 | PA0~PA3 | **PB0~PB15**（16 根，用满整个 B 口） |
| DRDY | 无（纯轮询） | 16 路经 **2×74HC30 + 1×74HC132** 合并 → **PA0 / EXTI0 下降沿** |
| 时钟 | APB2 = HCLK/4 = 18 MHz | **APB2 = HCLK/1 = 72 MHz** |
| 冷端补偿 | MAX31855 内部，线性 0.0625 °C/LSB | ADS1220 内部温度传感器（`TS=1`，14bit，0.03125 °C/LSB）+ **NIST ITS-90 多项式** |
| 协议 | `AA 55 LEN CMD ...` 定长 REC | **`55 CMD LEN DATA... CRC`（上行） / `AA CMD LEN DATA... CRC`（下行）** |

---

## 1. 新增文件

| 文件 | 内容 |
|---|---|
| `Core/Inc/board_config.h` | **单一事实来源**：片数/通道数、全部引脚映射、SPI 分频、全部时序常量（t_CLK / t_POWERUP / t_RESET / t_CSSC / t_SCCS / t_CSH / t_DATA / DR 等待窗口）、BCS 与失调校准开关 |
| `Core/Inc/spi_driver.h` + `Core/Src/spi_driver.c` | SPI1 总线层：16 路 CS 的 GPIO/EXTI/NVIC 初始化、带超时与自动恢复的事务收发、**合并 DRDY 的"只置标志位"中断接口**、DWT CYCCNT 微秒延时 |
| `Core/Inc/ads1220.h` + `Core/Src/ads1220.c` | ADS1220 驱动：6 条指令、4 个寄存器全部位域常量、24bit 读取、WREG/RREG、**16 片批量配置 + 回读校验**、**内部短路失调校准**、码值↔µV/°C 换算、DR 转换时间表 |
| `Core/Inc/thermocouple.h` + `Core/Src/thermocouple.c` | NIST ITS-90 K 型正/逆多项式（float + Horner）、冷端补偿、断线判定、上电自检 |
| `Core/Inc/uart_protocol.h` + `Core/Src/uart_protocol.c` | CRC-16/MODBUS、收帧状态机（帧内超时重同步）、中断发送环形缓冲、上行温度/状态帧组帧、下行命令解析与事件队列、协议自检 |
| `docs/TIMING.md` | 全部时序细节 + 需求 ↔ 数据手册逐条对照 + 串口时间预算 + 换算链 |
| `docs/BUILD.md` | Keil / Makefile 构建、CubeMX 重新生成后的检查清单、STM32F407 移植要点、故障排查表 |
| `Makefile` + `STM32F103xB_FLASH.ld` | arm-none-eabi-gcc 构建（主交付仍是 Keil 工程） |

## 2. 修改文件

| 文件 | 改动 |
|---|---|
| `Core/Src/main.c` | 全部应用逻辑重写：上电初始化序列、**四阶段非阻塞轮询状态机**、命令分发、冷端补偿、组帧上报、心跳与故障统计；`SystemClock_Config()` 的 APB2 改 `DIV1` |
| `Core/Src/spi.c` | `Direction: RXONLY → 2LINES`；`CLKPhase: 1EDGE → 2EDGE`（**Mode 0 → Mode 1**）；分频 `/32 → /16`；MSP 增加 **PA7 = SPI1_MOSI**；MISO 加上下拉（坏片识别） |
| `Core/Src/gpio.c` | 去掉 PA0~PA3 片选，改为调用 `SPI_Driver_GpioInit()`（16 路 CS + PA0 EXTI） |
| `Core/Src/usart.c` | 波特率改成 `UART_BAUDRATE` 宏（115200/460800）；RX 加上拉；`HAL_UART_MspInit()` 里使能 `USART1_IRQn` |
| `Core/Src/stm32f1xx_it.c` | 新增 `EXTI0_IRQHandler`（合并 DRDY，**只置标志位**）与 `USART1_IRQHandler` |
| `Core/Inc/stm32f1xx_it.h` | 新增两个中断处理函数原型 |
| `Core/Inc/main.h` | 增加固件名/版本号宏 |
| `Thermocouple.ioc` | 引脚重排：PA0=GPIO_EXTI0、PA5/6/7=SPI1 全双工、PB0~PB15=CS0~15（输出、初始高、High speed）；SPI1 改 2LINES + /16；APB2 改 DIV1；NVIC 加 EXTI0(1) 与 USART1(2)；栈从 0x400 提到 0x800 |
| `MDK-ARM/Thermocouple.uvprojx` | 把 `spi_driver.c` / `ads1220.c` / `thermocouple.c` / `uart_protocol.c` 加入编译列表 |
| `MDK-ARM/startup_stm32f103xb.s` | `Stack_Size 0x400 → 0x800`（浮点 + 128 字节帧缓冲） |
| `host_parser.py` | 改成**命令行外壳**（协议逻辑统一复用 `pc_ui/protocol.py`，不再重复一份 CRC/组帧代码） |
| `ReadMe.md` | 按新硬件/协议/算法重写 |

## 2.1 无需改动（已核对）

| 文件 | 说明 |
|---|---|
| `pc_ui/protocol.py` | **本来就是按新协议写的**（CRC-16/MODBUS、`55/AA CMD LEN DATA CRC`、32 路 float32、24 字节状态帧、DR/抑制索引），与本次固件实现**逐字节一致**，未作改动 |
| `pc_ui/serial_reader.py` | 协议无关（全部委托给 `pc_ui/protocol.py`），未作改动 |
| `pc_ui/__init__.py` | 新增（把 `pc_ui` 变成正式包，`python -m pc_ui.protocol` / `from pc_ui.protocol import ...` 更稳） |
| `Core/Src/stm32f1xx_hal_msp.c` | 已经做了 `AFIO + PWR` 时钟使能和 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`（SWD 调试），本设计不需要再改 |
| `Core/Src/system_stm32f1xx.c`、`stm32f1xx_hal_conf.h` | HAL 模块（SPI/GPIO/UART/EXTI/CORTEX/DMA/FLASH/PWR/RCC）本来就都开着，`HSE_VALUE = 8MHz` 也不用改 |

## 3. 删除文件

| 文件 | 原因 |
|---|---|
| `max31855.c` / `max31855.h` | 换成 ADS1220，根目录下这两个模板驱动不再需要，已删除 |

> `C52028_..._MAX31855...PDF`（旧传感器的规格书）**保留未删** —— 它是资料不是代码，
> 不影响编译，留着做对照也方便。要清理可以直接删掉。

---

## 4. 换芯片后必须改、且已经改掉的细节（重点）

这几条是"换芯片最容易漏"的地方，逐条确认：

1. **SPI 模式 Mode 0 → Mode 1**
   MAX31855 是 CPOL=0/CPHA=0；ADS1220 手册 6.6 节里
   `tsu(DI)/th(DI)` 相对 **SCLK 下降沿**定义、`tp(SCDO)` 是"SCLK **上升沿**到 DOUT 更新"，
   对应 **CPOL=0/CPHA=1**。HAL 里 `SPI_PHASE_1EDGE` → **`SPI_PHASE_2EDGE`**。
   ✅ 已在 `spi.c` 改掉并在注释里写了依据。

2. **从"只读"变成"要发指令" → 必须全双工 + 接 MOSI**
   `SPI_DIRECTION_2LINES_RXONLY` → `SPI_DIRECTION_2LINES`，MSP 里补上 **PA7 = SPI1_MOSI**。
   ✅ 已改。

3. **MAX31855 是 32bit 无指令流读，ADS1220 是 24bit + 寄存器**
   读法从"拉低 CS 直接锁 32 个 bit"变成
   "DRDY 下降沿 → 等 t_DATA → 拉低 CS → 锁 24 个 SCLK（3 字节）"，
   并且上电必须 RESET + WREG + START。
   ✅ `ads1220.c` 全部实现。

4. **寄存器位位置和需求文字不一致**（FIR 在 CONFIG2、50/60 同时抑制是 `01b` 不是 `11b`、
   DR 在 CONFIG1、连续转换是 CM 位、DRDY 由 CONFIG3.DRDYM 控制）
   ✅ 按数据手册落在正确位置，`docs/TIMING.md` §10 有完整对照表。

5. **"SELF CAL 0x04" 这条指令不存在**
   ✅ 换成手册 9.1.5 的内部短路失调校准（MUX=`1110b` 采 4 次平均）。

6. **冷端补偿算法完全不同**
   MAX31855 是芯片内部线性换算（12bit，0.0625 °C/LSB）；
   ADS1220 **不做冷端补偿**（手册 9.2.1.2 明确要求 MCU 自己做），
   必须"测 V_TC → 测 T_CJ → 正多项式求 E_CJ → 相加 → 逆多项式反解"。
   ✅ `thermocouple.c` 用 NIST ITS-90 系数实现。

7. **采样从"每 500 ms 轮询 4 片"变成"由 DRDY 中断驱动 16 片批量"**
   且 16 片共享一根合并中断，必须用"广播 START 重同步 + 时间门限"策略。
   ✅ `main.c` 的四阶段状态机。

8. **引脚全部重排**：片选 PA0~PA3 → **PB0~PB15**；PA0 让给合并 DRDY。
   ✅ `board_config.h` / `gpio.c` / `.ioc` 三处一致（`.ioc` 也改了，
   否则以后 CubeMX 重新生成会把 PA0 变回输出、把 SPI1 变回 RX-Only）。

9. **时钟树**：APB2 从 `HCLK/4 = 18 MHz` 改成 `HCLK/1 = 72 MHz`，
   这样 SPI1 = 72/16 = 4.5 MHz（满足 ADS1220 `tc(SC) ≥ 150 ns`），
   同时 USART1 拿 72 MHz，115200 和 460800 都是**零误差整数分频**。
   ✅ `main.c` / `.ioc` 都改了。

10. **协议完全换了**
    上行从 `AA 55 LEN CMD SEQ COUNT REC... CRC16-CCITT TAIL` 换成
    `55 CMD LEN DATA... CRC16-MODBUS`；下行新增 `AA CMD LEN DATA... CRC`。
    ✅ `uart_protocol.c` 实现，`host_parser.py` 同步重写。

11. **中断从"没有"变成"必须有两个"**
    `EXTI0`（合并 DRDY，优先级 1）+ `USART1`（优先级 2），
    且 EXTI0 的 ISR 里**绝不能有 SPI**。
    ✅ `stm32f1xx_it.c`。

12. **栈从 1 KB 提到 2 KB**
    新增了浮点运算（NIST 多项式）和 128 字节温度帧缓冲。
    ✅ `startup_stm32f103xb.s` 和 `.ioc` 都改了。

13. **SPI 分频数值**：`/32`（562.5 kHz，为 MAX31855 的 5 MHz 上限服务）
    → `/16`（4.5 MHz），并且把宏抽到 `board_config.h` 的
    `ADS1220_SPI_BAUDRATE_PSC`，避免以后改时钟树时漏改。
    ✅ 已改。

14. **`HAL_SPI_MspInit` 里 MISO 的上下拉**
    MAX31855 只有 4 片、DOUT 直连；现在 16 片共享 MISO。
    加了**下拉**，让"某片没焊/没响应"时读回确定的全 0，
    配合 `ADS1220_VerifyAll()` 的逐字节比对就能定位坏片。
    ✅ 已改。

---

## 5. 设计取舍说明

| 取舍 | 决定 | 理由 |
|---|---|---|
| SCLK 频率 | **4.5 MHz**（不是需求写的 10 MHz） | 手册 `tc(SC) ≥ 150 ns` → 上限 6.67 MHz；4.5 MHz 是 72 MHz 的整数分频，48% 裕量 |
| 50/60 Hz 抑制位 | **`01b`（`0x10`）** 不是需求写的 `11b` | `11b` 只是"仅抑制 60 Hz"；`01b` 才是"同时抑制 50+60 Hz" |
| BCS（断线电流源） | 默认**常开**（严格按需求） | 但 `docs/TIMING.md` §9 说明了 `10 µA × 回路电阻` 的失调影响，以及怎么改成按需开启 |
| 读数据方式 | **不發 RDATA，直接锁 24 个 SCLK** | 手册 9.1.6 伪代码和 Figure 8-25 就是这么写的；少 1 个字节，且连续模式下 DRDY 一低数据就绪 |
| 每阶段都发 START/SYNC | 是 | 合并 DRDY 分辨不出片号，只能靠 START 让 16 片重新对齐；代价是每阶段要等一个完整转换周期 |
| 等待门限 0.9 × 转换周期 | 是 | 防止把切换 MUX 之前的旧数据当成新通道数据 |
| 增益 | `GAIN=1 + PGA 旁路`（按需求） | 共模范围最宽、适合接地热电偶；代价是只用了满量程的 1.2%。改用 GAIN=32 的说明写在 `ReadMe.md` §1.4 |
| CRC | **CRC-16/MODBUS**，低字节在前 | 与固件/上位机统一，标准向量 `0x4B37` 便于双方自检 |
| 无效通道 | float32 **NaN** (`0x7FC00000`) | 温度帧保持"就是 32 个 float"的干净格式，故障细节放状态帧 |
| MCU | 保持 **STM32F103C8** | 合并 DRDY 后只需 24 个 IO，LQFP48 的 35 个可用 IO 够用；F407 移植清单在 `docs/BUILD.md` §5 |

---

## 6. 验证记录

| 项 | 方法 | 结果 |
|---|---|---|
| NIST K 型多项式 | 用 Python 按 IEEE-754 单精度仿真完整的 `t→E→t` 与冷端补偿闭环 | 正多项式对 6 个标准点误差 < 0.09 µV；逆多项式回环误差 < 0.06 °C；冷端补偿闭环 1000 °C → 999.98 °C ✅ |
| 协议 CRC / 组帧 / 拆帧 | `python host_parser.py --test`（与固件同一套算法） | 6 组共 22 项全部通过 ✅ |
| **固件帧布局 ↔ 上位机解析 交叉验证** | 按 `uart_protocol.c` 的字节顺序手工构造 24 字节状态帧与 133 字节温度帧（含 NaN / 负温 / 边界值），交给仓库里**原有**的 `pc_ui/protocol.py` 解析；再把 `pc_ui` 组的下行命令交给复刻的固件状态机解析 | **24 项全部通过**：字段偏移、小端、chip_ok/chip_err 位图、dr_sps 反查、抑制文本、flags 文本、32 路 float32、NaN 全部一致 ✅ |
| 寄存器配置字节 | 手工按 Table 8-8 位域核算 `ADS1220_BuildRegs()` | CONFIG0=0x01 / CONFIG1=0x05 / CONFIG2=0x10 / CONFIG3=0x00 ✅ |
| 时序常量 | 逐条对照 SBAS501D 6.6 / 8.3.7 / 8.3.12 / 8.3.13 / 8.4.1 / 8.5.3 | 见 `docs/TIMING.md` ✅ |
| 编译期时序自检 | `board_config.h` §6.1 的 4 条 `#if`（SPI 频率上下限、CS 建立 NOP 数、片选位宽），在 Python 里按同样的整数运算复算 | 4 条全部通过（4.5 MHz ≤ 6.67 MHz；8 NOP = 111 ns ≥ 50 ns）✅ |
| 代码结构 | 自写检查脚本：括号/花括号配平、`#include` 目标存在、头文件声明的函数都有定义（78 个声明 vs 114 个定义）、include guard 唯一、`#if/#endif` 配平、项目前缀标识符全部有定义 | 0 个问题 ✅ |
| C 代码编译 | 本机没有 arm-none-eabi / Keil / clang **任何** C 编译器，**未能实际编译** | 已逐文件人工复查接口闭合、类型、宏、头文件包含，并用脚本做了结构检查 ⚠️ |

> ⚠️ **唯一未完成的验证是"实际编译一次"** —— 这台机器上没有任何 C 编译工具链。
> 请第一次编译时留意；如果报错，基本都是拼写/包含路径这类问题，改起来很快。
> 另外用户第二条需求在 "### 主循环逻辑" 处被截断，
> **通信协议部分按第一条需求的完整描述实现**（`55 CMD LEN DATA CRC` / `AA CMD LEN DATA CRC`、
> CMD `0x10/0x11/0x01/0x02/0x03`），并且与仓库里原有的 `pc_ui/protocol.py` 完全吻合。

---

## 7. 后续可扩展方向

* 增加热电偶类型（J/T/N/E/R/S/B）：`thermocouple.c` 里再加一组 NIST 系数表，
  用命令 `CMD=0x04` 选择；`TC_Config_t` 里加一个 `type` 字段即可。
* 通道级增益：现在 16 片共用一套配置镜像；如果各片前置电路不同，
  把 `s_cfg` 改成 `s_cfg[16]` 并让 `SetConfigAll` 逐片下发即可。
* 数据流模式：把温度帧改成 `HAL_UART_Transmit_DMA` + 双缓冲，可以撑到更高上报率。
* 断线检测单独相位：把 `BCS` 改成"只在专用检测相位打开"，
  彻底消除 `10 µA × R` 的失调（目前默认常开是遵循需求）。
