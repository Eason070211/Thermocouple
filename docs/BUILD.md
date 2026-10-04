# 构建与工程配置说明

工程有两种构建方式，**源码完全一致**：

| 方式 | 入口 | 说明 |
|---|---|---|
| Keil MDK-ARM（主推） | `MDK-ARM/Thermocouple.uvprojx` | 已把新增的 4 个 .c 加进 `<Group>Application/User/Core</Group>` |
| arm-none-eabi-gcc | 根目录 `Makefile` + `STM32F103xB_FLASH.ld` | 无 Keil 环境时用，适合 CI / 命令行 |

---

## 1. Keil MDK-ARM

### 1.1 打开与编译

直接双击 `MDK-ARM/Thermocouple.uvprojx`，`F7` 编译，`F8` 下载。

已确认的工程设置（`Options for Target`）：

| 项 | 值 | 备注 |
|---|---|---|
| Device | `STM32F103C8` | LQFP48, 64 KB Flash / 20 KB RAM |
| C/C++ → Define | `USE_HAL_DRIVER,STM32F103xB` | **必须**，否则 `board_config.h` 会报 `#warning` |
| C/C++ → Include Paths | `../Core/Inc;../Drivers/STM32F1xx_HAL_Driver/Inc;../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy;../Drivers/CMSIS/Device/ST/STM32F1xx/Include;../Drivers/CMSIS/Include` | 不变 |
| C/C++ → C99 | ✔ | 代码用了块内声明 |
| Debug | SWD (`SWDIO=PA13`, `SWCLK=PA14`) | 不要选 JTAG，PA15/PB3/PB4 已被 CS 占用 |
| 栈大小 | `startup_stm32f103xb.s` 里 `Stack_Size EQU 0x800` | 已从 0x400 提到 2 KB |

> ⚠️ **如果 Keil 报 "not enough information to list image symbols" 之类**，
> 先 `Rebuild all`。新增 4 个源文件已在工程里，不需要手工添加。

### 1.2 加入工程的文件

```
Application/User/Core
├── main.c            非阻塞轮询状态机 / 命令分发 / 上报
├── gpio.c            GPIO 壳, 调用 SPI_Driver_GpioInit()
├── spi.c             SPI1 外设初始化 (Mode1, 2LINES, /16)
├── usart.c           USART1 初始化 (115200/460800) + NVIC
├── spi_driver.c      ★ 总线层: CS 时序 / SPI 事务 / 每片 DRDY 轮询 / DWT 延时
├── ads1220.c         ★ ADS1220 驱动: 寄存器 / 指令 / 24bit 读 / 失调校准
├── thermocouple.c    ★ NIST ITS-90 K 型正逆多项式 + 冷端补偿 + 断线判定
├── uart_protocol.c   ★ 协议: CRC16 / 收帧状态机 / 中断发送环形缓冲
├── stm32f1xx_it.c    USART1 中断 (DRDY 已改为轮询, 无 EXTI)
└── stm32f1xx_hal_msp.c
```

---

## 2. arm-none-eabi-gcc

```bash
make -j8          # 产出 build/Thermocouple.elf / .hex / .bin
make clean
make flash        # 需要 st-flash (stlink 工具)
```

需要安装 Arm GNU Toolchain，并保证 `arm-none-eabi-gcc` 在 PATH 里。
链接脚本 `STM32F103xB_FLASH.ld` 用 `-lm`（`thermocouple.c` 用到 `expf()`）。

---

## 3. 关于 `Thermocouple.ioc`（CubeMX）

`.ioc` 已按新拓扑改好，**如果以后再生成代码，下面的设置不要动**：

| 外设/引脚 | 设置 |
|---|---|
| `PA0`~`PA3` | `GPIO_Input`，Pull-up，标签 `DRDY0`~`DRDY3` |
| `PA5/PA6/PA7` | `SPI1_SCK` / `SPI1_MISO` / `SPI1_MOSI`，Mode = `Full_Duplex_Master` |
| `PA9/PA10` | `USART1_TX` / `USART1_RX`，Asynchronous |
| `PA13/PA14` | `SYS_JTMS-SWDIO` / `SYS_JTCK-SWCLK` |
| `PB0`~`PB3` | `GPIO_Output`，Speed = High，Initial Level = **High**，标签 `CS0`~`CS3` |
| `SPI1` | Direction = `2LINES`，Master，`BaudRatePrescaler = 16` → 4.5 MBits/s |
| `RCC` | HSE 8 MHz，PLL ×9 = 72 MHz，**APB2 = HCLK/1 = 72 MHz** |
| `NVIC` | `USART1_IRQn` 抢占优先级 2（DRDY 用轮询，**无 EXTI 中断**） |

### 重新生成后必须确认的 4 件事

CubeMX 会重写 `gpio.c` / `spi.c` / `usart.c`，但它**保留 `USER CODE` 区**。
本工程把所有"容易丢"的初始化都塞进了 `USER CODE` 或独立模块，重新生成后检查：

1. `Core/Src/gpio.c` 的 `MX_GPIO_Init()` 末尾 `USER CODE BEGIN 2` 里
   那一行 `SPI_Driver_GpioInit();` **还在**（4 路 CS(PB0..PB3) + 4 路 DRDY(PA0..PA3) 全靠它）。
2. `Core/Src/spi.c` 的 `MX_SPI1_Init()` 里
   `Direction = SPI_DIRECTION_2LINES`、`CLKPhase = SPI_PHASE_2EDGE`、
   `BaudRatePrescaler = ADS1220_SPI_BAUDRATE_PSC`。
   （CubeMX 不会知道"ADS1220 要 Mode 1"，重新生成后要人工核对。）
3. `Core/Src/usart.c` 里 `huart1.Init.BaudRate = UART_BAUDRATE;`
   和 `HAL_UART_MspInit()` 里的 `HAL_NVIC_EnableIRQ(USART1_IRQn)`。
4. `Core/Src/stm32f1xx_it.c` 的 `USER CODE BEGIN 1` 里
   `USART1_IRQHandler()` 还在（EXTI0 已移除，不会再生成 `EXTI0_IRQHandler`）。

> 换句话说：**即使你完全不用 CubeMX 重新生成，现有文件也能直接编译**；
> 上面的检查清单只是给"以后要加外设、必须重新生成"的情况准备的。

---

## 4. 时钟树与波特率/SPI 速率的由来

```
HSE 8 MHz ──► PLL ×9 ──► SYSCLK = 72 MHz ──► HCLK = 72 MHz (AHB /1)
                                              ├── APB1 = 36 MHz  (HCLK /2, 上限 36 MHz)
                                              └── APB2 = 72 MHz  (HCLK /1, 上限 72 MHz)
                                                    ├── SPI1  → /16 = 4.5 MHz  (222 ns ≥ 150 ns ✔)
                                                    └── USART1
```

| 波特率 | BRR 计算 | 误差 |
|---|---|---|
| 115200 | 72e6 / 115200 = 625.0000 | **0 %** |
| 460800 | 72e6 / 460800 = 156.2500 → 小数部分 4/16 | **0 %** |

> **注意**：原模板把 APB2 设成 `HCLK/4 = 18 MHz`（为了把 MAX31855 的 SCK 压到
> 562.5 kHz）。本工程已在 `SystemClock_Config()` 和 `.ioc` 里改成 `HCLK/1 = 72 MHz`，
> 这样 SPI1 和 USART1 都能拿到干净的整数分频。**如果沿用 18 MHz，
> 波特率仍然能算准，但 SPI 分频要改成 /4，容易漏改，所以统一到 72 MHz。**

---

## 5. 换成 STM32F407 需要改什么

代码已经把 MCU 相关的分支用 `BOARD_MCU_FAMILY_F1` / `BOARD_MCU_FAMILY_F4`
（`board_config.h` 用 HAL 头文件的 include guard 自动判定）隔离好了。
引脚拓扑（CS=PB0~PB3、DRDY=PA0~PA3 每片一根）**不需要变**。

要改的只有这些：

| 项 | F103C8（当前） | F407 |
|---|---|---|
| HAL 驱动包 | `Drivers/STM32F1xx_HAL_Driver` | `Drivers/STM32F4xx_HAL_Driver` |
| CMSIS 器件头 | `STM32F1xx/Include` | `STM32F4xx/Include` |
| 启动文件 | `startup_stm32f103xb.s` | `startup_stm32f407xx.s` |
| C/C++ Define | `USE_HAL_DRIVER,STM32F103xB` | `USE_HAL_DRIVER,STM32F407xx` |
| 系统文件 | `system_stm32f1xx.c` | `system_stm32f4xx.c` |
| `SystemClock_Config()` | HSE×9=72 MHz，APB2=/1 | HSE×PLL → 168 MHz，APB2=/2=84 MHz |
| SPI 分频 | `/16` → 72/16 = **4.5 MHz** | `/16` → 84/16 = **5.25 MHz**（仍 ≤6.67 MHz ✔，宏不用改） |
| GPIO 高速档 | `GPIO_SPEED_FREQ_HIGH` = 50 MHz | `GPIO_SPEED_FREQ_VERY_HIGH`（100 MHz）；`HIGH` 只有 25 MHz |
| SWD 引脚 | PA13/PA14 | PA13/PA14（相同） |
| `stm32f1xx_it.c/h` | 文件名带 f1 | 改成 `stm32f4xx_it.c/h`，异常处理名称一致 |

`spi_driver.c` 里已经写好了 `#if defined(BOARD_MCU_FAMILY_F1) / #else` 两个分支，
**应用层（`ads1220.c` / `thermocouple.c` / `uart_protocol.c` / `main.c`）一行都不用改。**

> 反过来：**STM32F103C8 不改，就用当前这一份**。4 路 CS(PB0~PB3) + 4 路 DRDY(PA0~PA3)
> + SPI1(3) + USART1(2) + SWD(2) 共 **15 个 IO**，LQFP48 的 35 个可用 IO 完全够
> （剩余 20 个可留作指示灯/蜂鸣器/预留）。

---

## 6. 上电后应该看到什么

1. 上电 → 约 50 ms 电源等待 → 4 片 RESET → 1 ms → 写 4 个寄存器 → 回读校验
   → 4 次失调校准（约 210 ms）→ START。
2. 之后每约 150 ms 通过 USART1 吐出 **133 字节**的上行温度帧（`55 10 80 ...`）。
3. 每 5 s 自动推一帧状态帧（`55 11 18 ...`），也可以在收到命令后立即回一帧。

用附带的上位机脚本验证：

```bash
python host_parser.py --test          # 先跑协议自检 (不需要硬件)
python host_parser.py COM3            # 打开串口实时打印温度
python host_parser.py COM3 --stop     # 停止采集 (状态帧里 run=0)
python host_parser.py COM3 --set-rate 0 1   # 20SPS + 同时抑制 50/60Hz
python host_parser.py COM3 --single 5       # 单次读第 5 通道
```

**排查顺序**（出问题时）：

| 现象 | 先查什么 |
|---|---|
| 完全没有串口输出 | PA9/PA10 接线、波特率、`USART1_IRQn` 是否使能、是否 `App_Start()` 被调用 |
| 有输出但 8 路全是 `----`(NaN) | 状态帧 `chip_ok` 位图。全 0 → SPI 没通（SCK/MOSI/MISO 接反、共地、SPI Mode 配成了 Mode 0） |
| `chip_ok` 只有部分位是 1 | `chip_err` 位图 + 对应片子的 CS 焊点；那几片的 MISO 没接上或被短路 |
| 状态帧 `flags` 有 `DRDY静默(0x02)` | 每片 DRDY 有没有接对 (PA0~PA3)、没焊/断路的片是否被上拉钳高 |
| 温度数值离谱 | 冷端传感器读到的不是环境温度 → 检查 `TS` 位切换、以及 CONFIG0 在 TS=1 时无效这一点 |
| 断线报太多 | `BCS` 常开导致的失调（见 `docs/TIMING.md` §9），把 `TC_BCS_ALWAYS_ON` 改 0 |
