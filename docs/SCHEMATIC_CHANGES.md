# 原理图变更记录 — 4×ADS1220 / 8 路热电偶

> 日期：2026-10-03
> 变更依据：`docs/HARDWARE_8CH.md`（8 路版本硬件设计说明）
> 原理图已备份（用户确认）

---

## 1. MCU 引脚重新分配

原设计（16 路方案）把 PB0–PB15 全部用作 CS，PA0 用作合并 DRDY。
改为 4 片后按 `HARDWARE_8CH.md` §1 重新分配：

| MCU 引脚 | 原功能 | 新功能 | 网络名 | 备注 |
|---|---|---|---|---|
| PA5 | 空闲 | SPI1_SCK | `SCLK` | 4 片并联 |
| PA6 | 空闲 | SPI1_MISO | `DOUT` | 4 片并联，片选高时高阻 |
| PA7 | 空闲 | SPI1_MOSI | `DIN` | 4 片并联 |
| PA0 | 空闲 | DRDY0 | `DRDY_0` | U8 数据就绪 |
| PA1 | 空闲 | DRDY1 | `DRDY_1` | U9 |
| PA2 | 空闲 | DRDY2 | `DRDY_2` | U10 |
| PA3 | 空闲 | DRDY3 | `DRDY_3` | U11 |
| PB0 | 空闲 | CS0 | `CS_0` | U8 片选 |
| PB1 | 空闲 | CS1 | `CS_1` | U9 |
| **PB2** | **BOOT1** | **CS2** | `CS_2` | ⚠️ 原为 BOOT1，已释放 |
| PB3 | 空闲 | CS3 | `CS_3` | ⚠️ 原为 JTDO，固件已关 JTAG |
| PA9 | 空闲 | USART1_TX | `TX` | 不变 |
| PA10 | 空闲 | USART1_RX | `RX` | 不变 |
| PA13 | 空闲 | SWDIO | `PA13` | 不变 |
| PA14 | 空闲 | SWCLK | `PA14` | 不变 |

### BOOT1 释放说明

- PB2 原为 BOOT1（启动模式选择），配 R58（10k 下拉到 GND）
- 改为 CS2 后，BOOT1 功能丧失
- **影响**：无法通过 BOOT0=1 + BOOT1=0 进入 USB DFU 模式
- **缓解**：用户使用 SWD 下载，不依赖 USB DFU
- **已删除**：R58（BOOT1 下拉电阻）
- **注意**：如未来需要 USB DFU，需将 CS2 挪到其他引脚（如 PB4）

---

## 2. ADS1220 模拟前端变更

### 2.1 删除串联电阻（6.7kΩ → 0Ω → 删除）

原设计在每路热电偶输入串 6.7kΩ 电阻（R33/R34/R42–R55）。
与 BCS（烧断检测）冲突：10µA × 6.7kΩ = 67mV → K 型 ≈ 1600°C 误差。

**决策**：删除串联电阻，改用纯电容滤波 + 软件滤波。
**固件配合**：`TC_BCS_ALWAYS_ON` 保持 `1`（BCS 正常工作）。

| 已删除电阻 | 对应通道 |
|---|---|
| R33, R34 | TC0/TC1 |
| R42, R43 | TC2 |
| R44, R45 | TC3 |
| R46, R47 | TC4 |
| R48, R49 | TC5 |
| R50, R51 | TC6 |
| R52, R53 | TC7 |
| R54, R55 | TC8（孤儿） |

### 2.2 滤波电容保留

每通道保留：
- 2 × 1µF 对地电容（共模滤波）
- 1 × 100nF 差分电容

### 2.3 REFP0/REFN0 处理

- **REFN0**：接 DGND（4 片均已连接）✓
- **REFP0**：原接 3V3_REF（悬空网络），已断开
- **原因**：使用片内 2.048V 基准，外部基准不用
- **状态**：REFP0 现在悬空（按数据手册，不用时可悬空或接 AVDD）

---

## 3. 网络连接方式

本原理图采用 **网络名合并** 方式连接（同名网络自动合并），
而非物理导线连续。这是 Altium 导入后的特征。

关键网络：
- `CS_0`–`CS_3`：MCU PB0–PB3 ↔ ADS1220 各片 CS#
- `DRDY_0`–`DRDY_3`：MCU PA0–PA3 ↔ ADS1220 各片 DRDY#
- `SCLK`：MCU PA5 ↔ 4 片 SCLK（并联）
- `DOUT`：MCU PA6 ↔ 4 片 DOUT/DRDY#（并联）
- `DIN`：MCU PA7 ↔ 4 片 DIN（并联）
- `TC0_P/N`–`TC7_P/N`：热电偶端子 ↔ ADS1220 AIN

---

## 4. 已完成 / 待手动补全

### ✅ 已完成

| 项 | 说明 |
|---|---|
| MCU ↔ ADS1220 信号连接 | CS_0–3, DRDY_0–3, SCLK/DOUT/DIN 全部接通 |
| 热电偶端子接线 | H1→TC0, H13→TC2, H14→TC3, H15→TC4, H16→TC5, H17→TC6, H18→TC7 |
| 6.7k 串联电阻删除 | R33/R34/R42–R55 全部删除 |
| TC8 孤儿电容删除 | C78/C79/C84 已删 |
| REFP0 3V3_REF 清理 | 4 个 flag 全部断开 |
| R58 删除 | BOOT1 下拉电阻已移除 |

### ⚠️ 需要你在 GUI 手动补全

**TC0 滤波电容**（H1 已接 TC0，但缺电容）：
- 在 H1 附近放 2 个 1µF 0603 电容：一个接 TC0_P→AGND，一个接 TC0_N→AGND
- 再放 1 个 100nF 0603 差分电容：跨接 TC0_P↔TC0_N
- 位置参考：H1 在 (1250,-370)，电容放右侧即可

**TC1 端子**（TC1 有电容 C13/C14/C15 但没端子）：
- 放一个 2P 2.54mm 排针，pin1 接 TC1_P，pin2 接 TC1_N
- 位置参考：U8 在 (1760,805)，TC1 引脚在右侧

> **原因**：API 创建组件持续超时（30s），无法通过脚本添加新器件。

### 其他

| 项 | 状态 | 说明 |
|---|---|---|
| MISO 100k 下拉 | 未加 | 规格建议，分批贴装时需要 |
| 47Ω 串联电阻 | 不加 | 用户决定 |

---

## 5. 固件侧需同步修改

| 文件 | 修改 | 原因 |
|---|---|---|
| `Core/Inc/board_config.h` | `ADS1220_CHIP_COUNT` = `4` | 4 片 |
| `Core/Inc/board_config.h` | `TC_BCS_ALWAYS_ON` = `1` | 无串联电阻，BCS 正常 |
| `Core/Src/spi_driver.c` | CS 引脚改为 PB0–PB3（4 根） | 原为 PB0–PB15 |
| `Core/Src/spi_driver.c` | DRDY 改为 PA0–PA3 独立（无合并逻辑） | 原为 PA0 EXTI0 |
| `Core/Src/stm32f1xx_it.c` | 删除 EXTI0 中断处理 | DRDY 不再合并 |
| `Core/Src/spi.c` | 确认 SPI Mode 1、全双工 | 不变 |
