/**
  ******************************************************************************
  * @file    thermocouple.h
  * @brief   K 型热电偶换算 (NIST ITS-90 多项式) + 冷端补偿
  *
  *  === 算法 (完全按 TI ADS1220 数据手册 9.2.1.2 的冷端补偿步骤) ===
  *    1. 测出热电偶两端的电压 V_TC            (ADS1220 差分通道, µV)
  *    2. 用 ADS1220 内部温度传感器测出冷端温度 T_CJ
  *    3. 用 NIST 正多项式把 T_CJ 换算成等效热电势 V_CJ = E(T_CJ)
  *    4. 求和 V = V_TC + V_CJ, 再用 NIST 逆多项式反解出热端温度
  *         T_TC = E^-1(V_TC + V_CJ)
  *
  *  === 多项式来源 ===
  *    NIST Monograph 175 / ITS-90 附表 (K 型):
  *      正多项式 (t -> mV):
  *         -270°C ~    0°C   : E = Σ c_i t^i                     (10 阶)
  *            0°C ~ 1372°C   : E = Σ c_i t^i + a0 exp(a1 (t-a2)^2) (9 阶 + 指数项)
  *      逆多项式 (µV -> °C):
  *         -200°C ~    0°C   : t = Σ d_i E^i     E ∈ [-5891, 0]     µV
  *            0°C ~  500°C   : t = Σ d_i E^i     E ∈ [0, 20644]     µV
  *          500°C ~ 1372°C   : t = Σ d_i E^i     E ∈ [20644, 54886] µV
  *
  *  === 精度 ===
  *    全部用 float (单精度) + 秦九韶(Horner)展开。在 F103(无 FPU) 上单精度比
  *    double 快数倍, 实测误差 < 0.03°C, 远好于热电偶本身和冷端传感器的误差。
  *    (见 TC_SelfTest(), 上电时可以跑一遍)
  ******************************************************************************
  */

#ifndef __THERMOCOUPLE_H
#define __THERMOCOUPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "board_config.h"

/*==============================================================================
 * K 型热电偶物理量程
 *============================================================================*/
#define TC_K_TYPE_NAME        "K"
#define TC_K_TEMP_MIN_C       (-200.0f)     /**< 可测热端下限 °C */
#define TC_K_TEMP_MAX_C       (1372.0f)     /**< 可测热端上限 °C */
#define TC_K_EMF_MIN_UV       (-5891.0f)    /**< E(-200°C), 逆多项式第一段边界 µV */
#define TC_K_EMF_MID_UV       (20644.0f)    /**< E( 500°C), 逆多项式第二段边界 µV */
#define TC_K_EMF_MAX_UV       (54886.0f)    /**< E(1372°C), 逆多项式第三段边界 µV */

/** 冷端温度的有效范围 (ADS1220 内部温度传感器工作范围, 也是等温块的范围) */
#define TC_CJ_MIN_C           (-40.0f)
#define TC_CJ_MAX_C           (125.0f)

/** 断线(开路)判定的热电势窗口 —— 见 board_config.h 的 TC_OPEN_EMF_*。
 *
 *  这里的物理依据: 合法范围 V_TC = E(T_TC) - E(T_CJ)。
 *    K 型 : T_TC∈[-200,1372], T_CJ∈[-40,125] => 约 [-9400, +56413] µV
 *    最宽的是 E 型 (1000°C 约 76373 µV), 所以默认窗口取 ±80mV, 见 board_config.h。
 *
 *  ★ 这一条是**分度号相关**的辅助判据; 主判据是"码值被 BCS/偏置电阻拉死到
 *    ±满量程"(与分度号无关, 见 TC_ADC_CODE_*)。所以 TC_OPEN_EMF_CHECK=0
 *    关掉它也不会漏判断线, 只是少了一层"高阻未满量程"的兜底。 */
#define TC_K_EMF_OPEN_HI_UV   (TC_OPEN_EMF_HI_UV)
#define TC_K_EMF_OPEN_LO_UV   (TC_OPEN_EMF_LO_UV)

/** ADS1220 24bit 满量程码值 (用于识别"输入被拉死") */
#define TC_ADC_CODE_POS_FS    (0x7FFFFFL)
#define TC_ADC_CODE_NEG_FS    (-8388608L)

/*==============================================================================
 * 计算结果
 *============================================================================*/
typedef struct
{
  float   hot_c;      /**< 热端温度 °C (冷端补偿后) */
  float   cj_c;       /**< 冷端温度 °C (ADS1220 内部温度传感器) */
  float   emf_tc_uv;  /**< 实测热电势 µV (已减失调) */
  float   emf_cj_uv;  /**< 冷端补偿热电势 E(T_CJ) µV */
  uint8_t open;       /**< 1 = 断线/超量程 (hot_c 无效) */
  uint8_t valid;      /**< 1 = 数据有效 */
} TC_Result_t;

/*==============================================================================
 * API
 *============================================================================*/

/** 正多项式: 温度 °C -> 热电势 µV。
 *  @note 输入会被裁剪到 [-270, 1372] °C 以保证多项式不发散。 */
float TC_K_EmfMicroVolt(float temp_c);

/** 逆多项式: 热电势 µV -> 温度 °C。
 *  @note 输入会被裁剪到 [-5891, 54886] µV。 */
float TC_K_TempFromEmf(float emf_uv);

/** 冷端补偿: 由"实测热电势 + 冷端温度"算出热端温度 °C。 */
float TC_CompensateHotJunction(float emf_tc_uv, float cj_c);

/** ★ 只做断线/超量程判定, **不**换算温度。
 *
 *  给 TC_UPLINK_MODE=1 (温度由上位机算) 用: 固件仍然需要在本地判开路,
 *  因为状态帧要报 open_tc_count, 而且断线通道不该把 µV 发成"看起来正常"的值。
 *
 *  判定依据与 TC_Compute() 逐条一致, 但不含任何多项式运算:
 *    1) 冷端温度超出 ADS1220 内部温度传感器范围 -> 本片两路都无法计算;
 *    2) 码值被 BCS/偏置电阻拉死到 ±满量程 (断线最典型的表现);
 *    3) 热电势超出物理窗口 (TC_OPEN_EMF_*, 可用宏关掉)。
 *
 *  @param emf_tc_uv  已做失调补偿的差分电压 (µV)
 *  @param cj_c       冷端温度 (°C)
 *  @param raw_code   24bit 原始码值 (识别满量程); 没有就传 0
 *  @retval 1 = 开路/超量程 (该通道无效, 上位机应显示 NaN); 0 = 正常 */
uint8_t TC_IsOpen(float emf_tc_uv, float cj_c, int32_t raw_code);

/** 一次算完: 断线判定 + 冷端补偿 + 量程裁剪。
 *  @param emf_tc_uv  已做失调补偿的差分电压 (µV)
 *  @param cj_c       冷端温度 (°C)
 *  @param raw_code   24bit 原始码值 (用于识别满量程); 没有就传 0
 *  @param r          输出
 *  @note  r->open==1 时 r->hot_c 为无效值(置 0), 调用者应上报 NaN。 */
void TC_Compute(float emf_tc_uv, float cj_c, int32_t raw_code, TC_Result_t *r);

/** 自检: 用 NIST 标准点验证正/逆多项式与冷端补偿链路。
 *  @retval 1 = 通过; 0 = 失败 (多项式或编译器浮点有问题) */
uint8_t TC_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __THERMOCOUPLE_H */
