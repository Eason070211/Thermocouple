/**
  ******************************************************************************
  * @file    thermocouple.c
  * @brief   K 型热电偶 NIST ITS-90 多项式 + 冷端补偿实现
  *
  *  系数全部取自 NIST Monograph 175 (ITS-90 附表, K 型)。
  *  正多项式输出单位是 mV, 本文件对外统一用 µV (x1000)。
  *  逆多项式输入单位是 µV。
  *
  *  为什么要"先算冷端等效电势再加起来":
  *    热电偶输出的是 (T_hot - T_cold) 对应的电势差, 只有在冷端为 0°C 时才是
  *    直接对应 T_hot 的电势。所以必须把冷端温度换算成等效电势补回去。
  ******************************************************************************
  */

#include "thermocouple.h"
#include <math.h>

/*==============================================================================
 * NIST ITS-90 K 型系数
 *============================================================================*/

/** 正多项式 -270°C ~ 0°C  (mV = Σ c_i · t^i) */
static const float TC_K_C_LO[11] =
{
   0.000000000000E+00f,
   0.394501280250E-01f,
   0.236223735980E-04f,
  -0.328589067840E-06f,
  -0.499048287770E-08f,
  -0.675090591730E-10f,
  -0.574103274280E-12f,
  -0.310888728940E-14f,
  -0.104516093650E-16f,
  -0.198892668780E-19f,
  -0.163226974860E-22f
};

/** 正多项式 0°C ~ 1372°C  (mV = Σ c_i · t^i) */
static const float TC_K_C_HI[10] =
{
  -0.176004136860E-01f,
   0.389212049750E-01f,
   0.185587700320E-04f,
  -0.994575928740E-07f,
   0.318409457190E-09f,
  -0.560728448890E-12f,
   0.560750590590E-15f,
  -0.320207200030E-18f,
   0.971511471520E-22f,
  -0.121047212750E-25f
};

/* 0°C 以上多出来的指数修正项: a0 · exp(a1 · (t - a2)^2) */
static const float TC_K_A0 =  0.118597600000E+00f;
static const float TC_K_A1 = -0.118343200000E-03f;
static const float TC_K_A2 =  0.126968600000E+03f;

/** 逆多项式 -200°C ~ 0°C  (°C = Σ d_i · E^i, E 单位 µV) */
static const float TC_K_D_LO[9] =
{
   0.0000000E+00f,
   2.5173462E-02f,
  -1.1662878E-06f,
  -1.0833638E-09f,
  -8.9773540E-13f,
  -3.7342377E-16f,
  -8.6632643E-20f,
  -1.0450598E-23f,
  -5.1920577E-28f
};

/** 逆多项式 0°C ~ 500°C */
static const float TC_K_D_MID[10] =
{
   0.000000E+00f,
   2.508355E-02f,
   7.860106E-08f,
  -2.503131E-10f,
   8.315270E-14f,
  -1.228034E-17f,
   9.804036E-22f,
  -4.413030E-26f,
   1.057734E-30f,
  -1.052755E-35f
};

/** 逆多项式 500°C ~ 1372°C */
static const float TC_K_D_HI[7] =
{
  -1.318058E+02f,
   4.830222E-02f,
  -1.646031E-06f,
   5.464731E-11f,
  -9.650715E-16f,
   8.802193E-21f,
  -3.110810E-26f
};

/*==============================================================================
 * 私有函数
 *============================================================================*/
/**
  * @brief 秦九韶(Horner)法求多项式 c[0] + c[1]x + ... + c[n-1]x^(n-1)
  * @note  用 Horner 而不是逐项求幂, 既快(少一半乘法)又能把中间量控制在
  *        和结果同量级, 避免 NIST 逆多项式在大 E 处的灾难性抵消。
  */
static float TC_Horner(const float *c, uint8_t n, float x)
{
  float   acc;
  int16_t i;

  if ((c == NULL) || (n == 0u))
  {
    return 0.0f;
  }

  acc = c[n - 1u];
  for (i = (int16_t)n - 2; i >= 0; i--)
  {
    acc = (acc * x) + c[(uint8_t)i];
  }

  return acc;
}

/*==============================================================================
 * 正多项式: t (°C) -> E (µV)
 *============================================================================*/
float TC_K_EmfMicroVolt(float temp_c)
{
  float t = temp_c;
  float mv;

  /* 多项式在量程外会发散, 先裁剪 (K 型正多项式本身覆盖 -270 ~ 1372°C) */
  if (t < -270.0f)
  {
    t = -270.0f;
  }
  if (t > TC_K_TEMP_MAX_C)
  {
    t = TC_K_TEMP_MAX_C;
  }

  if (t < 0.0f)
  {
    mv = TC_Horner(TC_K_C_LO, 11u, t);
  }
  else
  {
    float d = t - TC_K_A2;
    mv = TC_Horner(TC_K_C_HI, 10u, t) + (TC_K_A0 * expf(TC_K_A1 * d * d));
  }

  return mv * 1000.0f;      /* mV -> µV */
}

/*==============================================================================
 * 逆多项式: E (µV) -> t (°C)
 *============================================================================*/
float TC_K_TempFromEmf(float emf_uv)
{
  float e = emf_uv;

  if (e < TC_K_EMF_MIN_UV)
  {
    e = TC_K_EMF_MIN_UV;
  }
  if (e > TC_K_EMF_MAX_UV)
  {
    e = TC_K_EMF_MAX_UV;
  }

  if (e < 0.0f)
  {
    return TC_Horner(TC_K_D_LO, 9u, e);
  }
  if (e < TC_K_EMF_MID_UV)
  {
    return TC_Horner(TC_K_D_MID, 10u, e);
  }
  return TC_Horner(TC_K_D_HI, 7u, e);
}

/*==============================================================================
 * 冷端补偿
 *============================================================================*/
float TC_CompensateHotJunction(float emf_tc_uv, float cj_c)
{
  float emf_cj = TC_K_EmfMicroVolt(cj_c);
  return TC_K_TempFromEmf(emf_tc_uv + emf_cj);
}

/*==============================================================================
 * 一次算完 + 断线判定
 *============================================================================*/
void TC_Compute(float emf_tc_uv, float cj_c, int32_t raw_code, TC_Result_t *r)
{
  float emf_cj;
  float sum;

  if (r == NULL)
  {
    return;
  }

  r->hot_c     = 0.0f;
  r->cj_c      = cj_c;
  r->emf_tc_uv = emf_tc_uv;
  r->emf_cj_uv = 0.0f;
  r->open      = 0u;
  r->valid     = 0u;

  /* ---- 1) 冷端温度必须落在 ADS1220 内部温度传感器的工作范围内 ---- */
  if ((cj_c < TC_CJ_MIN_C) || (cj_c > TC_CJ_MAX_C))
  {
    r->open = 1u;
    return;
  }

  /* ---- 2) 断线判定 (两种独立依据, 任一命中即判开路) ---- */
  /*  a) 码值被拉死到 ±满量程: 断线后 10uA 烧断电流源 / 偏置电阻把 AINP/AINN
   *     推到两极 (手册 9.2.1.2: "the biasing resistors pull the analog inputs to
   *     AVDD and AVSS... The ADC consequently reads a full-scale value")       */
  if ((raw_code >= TC_ADC_CODE_POS_FS) || (raw_code <= TC_ADC_CODE_NEG_FS))
  {
    r->open = 1u;
    return;
  }
  /*  b) 换算出的热电势超出 K 型物理量程
   *     (合法范围约为 [-9400, +56413] µV, 见 thermocouple.h 说明)            */
  if ((emf_tc_uv > TC_K_EMF_OPEN_HI_UV) || (emf_tc_uv < TC_K_EMF_OPEN_LO_UV))
  {
    r->open = 1u;
    return;
  }

  /* ---- 3) 冷端补偿: V = V_TC + E(T_CJ), 再反解热端温度 ---- */
  emf_cj = TC_K_EmfMicroVolt(cj_c);
  sum    = emf_tc_uv + emf_cj;

  r->emf_cj_uv = emf_cj;
  r->hot_c     = TC_K_TempFromEmf(sum);
  r->valid     = 1u;
}

/*==============================================================================
 * 自检
 *============================================================================*/
uint8_t TC_SelfTest(void)
{
  /* NIST 标准点: 温度 °C, 期望热电势 µV (t=0°C 基准) */
  static const float s_temp[]  = { -100.0f,    0.0f,   25.0f,  500.0f, 1000.0f, 1372.0f };
  static const float s_emf[]   = { -3553.63f,  0.0f, 1000.24f, 20644.29f, 41275.61f, 54886.36f };
  static const float s_tol_emf = 1.0f;    /* µV */
  static const float s_tol_temp = 0.2f;   /* °C (逆多项式本身有 ~0.05°C 误差) */

  float e;
  float back;
  float v;
  float tc;
  uint8_t i;

  /* --- 1) 正多项式对标准点 --- */
  for (i = 0u; i < 6u; i++)
  {
    e = TC_K_EmfMicroVolt(s_temp[i]);
    if (fabsf(e - s_emf[i]) > s_tol_emf)
    {
      return 0u;
    }

    /* --- 2) 逆多项式回环: t -> E -> t --- */
    back = TC_K_TempFromEmf(e);
    if (fabsf(back - s_temp[i]) > s_tol_temp)
    {
      return 0u;
    }
  }

  /* --- 3) 冷端补偿闭环: 热端 1000°C / 冷端 25°C 应还原 1000°C --- */
  v  = TC_K_EmfMicroVolt(1000.0f) - TC_K_EmfMicroVolt(25.0f);
  tc = TC_CompensateHotJunction(v, 25.0f);
  if (fabsf(tc - 1000.0f) > s_tol_temp)
  {
    return 0u;
  }

  /* --- 4) 冷端 0°C 时热端应等于直接反解 --- */
  v  = TC_K_EmfMicroVolt(500.0f);
  tc = TC_CompensateHotJunction(v, 0.0f);
  if (fabsf(tc - 500.0f) > s_tol_temp)
  {
    return 0u;
  }

  return 1u;
}
