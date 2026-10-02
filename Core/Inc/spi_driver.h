/**
  ******************************************************************************
  * @file    spi_driver.h
  * @brief   SPI1 总线层: 片选管理 / 事务收发 / 每片 DRDY 轮询 + 软件合成 / 延时
  *
  *  职责边界:
  *    - 只关心"怎么在 SPI1 上安全地跟某一片 ADS1220 说一句话"
  *    - 不关心 ADS1220 的寄存器含义 (那是 ads1220.c 的事)
  *
  *  提供的功能:
  *    SPI_Driver_GpioInit()   CS(PB0..PBn) + DRDY(PA0..PA3, 每片一根) 的 GPIO 初始化
  *    SPI_Driver_Transfer()   选中片 -> tCSSC -> 收发 -> tSCCS -> 释放片 -> tCSH
  *    SPI_Driver_DrdyXxx()    DRDY 查询接口 (电平/掩码/软件合成的"全部就绪")
  *    SPI_Driver_DelayUs/Ms   DWT CYCCNT 精确微秒延时 (失败时退化为 NOP 循环)
  *
  *  === DRDY 方案变更 (16 片 -> 8 路) ===
  *  老版本: 16 片 DRDY 经 2x74HC30 + 74HC132 与非合并成一路 -> PA0/EXTI0 中断。
  *  新版本: 4 片各占一根 DRDY(PA0..PA3), 不用外部逻辑门, 由本层软件"合成":
  *
  *      全部就绪  <=>  (DRDY 引脚位图 & ADS1220_DRDY_MASK_ALL) == 0
  *
  *  这样做的好处:
  *    1) SPI_Driver_DrdyTakeFlag() / DrdyClearFlag() / DrdyLevel() 的**语义和原来
  *       完全一致**, 所以 ads1220.c 和 main.c 里已有的等待逻辑一个字都不用改;
  *    2) 额外提供 SPI_Driver_DrdyReadyMask(), 能分辨"到底是哪一片没就绪",
  *       超时时用来定位坏片 / 没焊的片;
  *    3) 没贴的芯片: 它的 DRDY 引脚被 MCU 内部上拉钳高 = 永远不"就绪";
  *       而固件只扫描 ADS1220_CHIP_COUNT 片, 所以没贴的片根本不在掩码里, 不拖慢采集。
  *
  *  与老 EXTI 方案的唯一行为差异:
  *    EXTI 是"下降沿锁存", 这里是"电平判定 + 全就绪锁存"。因此调用方必须保证:
  *    在 START/SYNC 之后先等过至少 0.9 个转换周期再调用 DrdyTakeFlag() ——
  *    main.c 的 App_StartWait()/App_WaitDone() 和 ads1220.c 的
  *    ADS1220_WaitDataReadyBlocking() 里本来就已经有这个最小等待, 无需改动。
  ******************************************************************************
  */

#ifndef __SPI_DRIVER_H
#define __SPI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "board_config.h"

/** 总线层返回码 (不直接用 HAL_StatusTypeDef, 便于区分"超时"和"参数错误") */
typedef enum
{
  SPI_DRV_OK = 0,          /**< 成功 */
  SPI_DRV_ERR_PARAM,       /**< 片号越界 / 空指针 / 长度为 0 */
  SPI_DRV_ERR_TIMEOUT,     /**< HAL 超时 (总线无响应) */
  SPI_DRV_ERR_HAL          /**< HAL 报了 BUSY/ERROR */
} SPI_Driver_Status_t;

/*----------------------------------------------------------------------------
 * 初始化 / 延时
 *--------------------------------------------------------------------------*/
/** 配置 CS(PB0..PBn, 推挽 50MHz, 初始高) 与 DRDY(PA0..PA3, 输入+内部上拉)。
 *  另外关闭 JTAG(保留 SWD), 否则 PB3/PB4 不能当普通输出用 (片数 >= 4 会用到)。
 *  在 MX_GPIO_Init() 里调用 (见 gpio.c), 这样 CubeMX 重新生成也不会丢。 */
void SPI_Driver_GpioInit(void);

/** 上电复位后 CS 全部拉高 (释放总线)。 */
void SPI_Driver_ReleaseAllCs(void);

/** 忙等延时。DelayUs 使用 DWT->CYCCNT, 精度 ~14ns @72MHz。 */
void SPI_Driver_DelayUs(uint32_t us);
void SPI_Driver_DelayMs(uint32_t ms);

/*----------------------------------------------------------------------------
 * SPI 事务
 *--------------------------------------------------------------------------*/
/**
  * @brief  对第 chip 片 ADS1220 做一次 SPI 事务
  * @param  chip  片号 0..ADS1220_CHIP_COUNT-1 (= PB0..PB3)
  * @param  tx    发送缓冲区, 必须非 NULL
  * @param  rx    接收缓冲区; 传 NULL 表示只写不读
  * @param  len   字节数 (1..255)
  * @retval SPI_Driver_Status_t
  * @note   内部自动处理 td(CSSC)/td(SCCS)/tw(CSH), 出错后自动恢复 SPI 外设。
  *         失败时 CS 仍会被正确释放, 不会把总线锁死。
  */
SPI_Driver_Status_t SPI_Driver_Transfer(uint8_t chip,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        uint16_t len);

/** 总线自恢复: 若 HAL SPI 状态卡在 BUSY, 中止并重新使能外设。 */
void SPI_Driver_Recover(void);

/*----------------------------------------------------------------------------
 * DRDY (每片一根 PA0..PA3, 软件合成"全部就绪")
 *
 *  下面 4 个函数的**语义与老的合并-DRDY 版本完全一致**, 老代码无需修改:
 *    DrdyTakeFlag()  : 1 = 全部被扫描的片都有新数据 (对应老的"合并 DRDY 下降沿")
 *    DrdyClearFlag() : 清掉"已见过全就绪"的锁存, 用于每次 START/SYNC 之前
 *    DrdyLevel()     : 0 = 全部就绪(低有效), 1 = 还有片没就绪
 *    DrdyReadyCount(): 累计"全部就绪"的次数 (原 DrdyIrqCount)
 *--------------------------------------------------------------------------*/
/** 读取并判断是否"全部就绪"; 返回 1 表示所有片都有未读走的数据。
 *  @note 由主循环/等待循环调用, 不在中断里使用 (新方案没有 DRDY 中断)。 */
uint8_t  SPI_Driver_DrdyTakeFlag(void);

/** 清掉"已见过全就绪"的锁存 (START/SYNC 之前调用)。 */
void     SPI_Driver_DrdyClearFlag(void);

/** 读"是否全部就绪"的当前电平。返回 0 表示低有效 = 所有片都有数据。 */
uint8_t  SPI_Driver_DrdyLevel(void);

/** 累计"全部就绪"的次数 (可用于判断各片是否都在正常转换)。 */
uint32_t SPI_Driver_DrdyReadyCount(void);

/** ★ 新增: 逐片就绪位图。bit n = 1 表示第 n 片 DRDY 已就绪(低)。
 *  用来在超时时定位"是哪一片没响应"(没焊/虚焊/供电异常)。 */
uint16_t SPI_Driver_DrdyReadyMask(void);

/** ★ 新增: 设置"本阶段需要等待就绪"的片掩码。
 *  main.c 在上电回读校验之后把它收窄成"实际存在的片", 这样:
 *    - 只贴了 2 片而 ADS1220_CHIP_COUNT 写成 4 时, 也不会因为缺片而每轮超时;
 *    - 某片中途坏掉时, 上层可以把它从等待集合里摘掉, 采集继续。
 *  @param mask 传 0 无效(会保持原值), 以免变成"永远就绪"。 */
void     SPI_Driver_DrdySetExpectedMask(uint16_t mask);

/** 读取当前"需要等待就绪"的片掩码。 */
uint16_t SPI_Driver_DrdyExpectedMask(void);

/** 累计的 SPI 事务失败次数 (超时/错误)。 */
uint32_t SPI_Driver_ErrorCount(void);

/** 最近一次 SPI 事务的失败片号 (0xFF = 无)。 */
uint8_t  SPI_Driver_LastErrorChip(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_DRIVER_H */
