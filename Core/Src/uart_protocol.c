/**
  ******************************************************************************
  * @file    uart_protocol.c
  * @brief   USART1 协议实现: 逐字节收帧状态机 + CRC16 + 中断发送环形缓冲
  *
  *  设计要点:
  *    - 收: 每来 1 字节进一次中断, 中断里只做"状态机 + 存字节", 不做任何耗时操作。
  *          帧内超时由主循环的 UART_Protocol_Poll() 负责复位。
  *    - 发: 环形缓冲 + HAL_UART_Transmit_IT, 主循环只把帧塞进缓冲立刻返回,
  *          不阻塞 (115200 下发一帧 133 字节要 11.5ms, 阻塞会顶掉 DRDY 处理)。
  *    - 帧同步: 只认帧头 0xAA, 配合 LEN 长度字段 + CRC 校验, 丢字节也能自动重新同步。
  ******************************************************************************
  */

#include "uart_protocol.h"
#include "usart.h"

/*==============================================================================
 * 私有变量
 *============================================================================*/
/*---- 收 ----*/
typedef enum
{
  RXS_HEAD = 0,
  RXS_CMD,
  RXS_LEN,
  RXS_DATA,
  RXS_CRC_LO,
  RXS_CRC_HI
} UART_RxState_t;

static uint8_t        s_rx_state;
static uint8_t        s_rx_buf[UART_FRAME_MAX_LEN];
static uint16_t       s_rx_idx;
static uint16_t       s_rx_need;
static uint32_t       s_rx_tick;
static uint8_t        s_rx_byte;      /* HAL 逐字节中断接收的落点 */
static uint8_t        s_rx_armed;     /* 1 = 接收中断已挂上 */

/*---- 事件队列 (4 深, 防止连发两条命令丢第二条) ----*/
#define UART_EV_QUEUE_LEN   4u
static UART_EventMsg_t s_ev_queue[UART_EV_QUEUE_LEN];
static volatile uint8_t s_ev_head;
static volatile uint8_t s_ev_tail;

/*---- 发 ----*/
static uint8_t           s_tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_chunk;
static volatile uint8_t  s_tx_busy;
static uint16_t          s_tx_overflow;

/*---- 统计 ----*/
static uint16_t s_crc_err_cnt;

/*==============================================================================
 * CRC-16/MODBUS
 *   多项式 0xA001 (0x8005 反射), 初值 0xFFFF, 输入/输出均反射, 无异或输出。
 *   标准校验向量: "123456789" -> 0x4B37
 *============================================================================*/
static uint16_t UART_CrcUpdate(uint16_t crc, uint8_t byte)
{
  uint8_t i;

  crc ^= (uint16_t)byte;
  for (i = 0u; i < 8u; i++)
  {
    if ((crc & 0x0001u) != 0u)
    {
      crc = (uint16_t)((crc >> 1) ^ 0xA001u);
    }
    else
    {
      crc = (uint16_t)(crc >> 1);
    }
  }

  return crc;
}

uint16_t UART_CRC16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  uint16_t i;

  if (data == NULL)
  {
    return crc;
  }

  for (i = 0u; i < len; i++)
  {
    crc = UART_CrcUpdate(crc, data[i]);
  }

  return crc;
}

/*==============================================================================
 * 小端打包
 *============================================================================*/
void UART_PutU16LE(uint8_t *buf, uint16_t v)
{
  if (buf == NULL)
  {
    return;
  }
  buf[0] = (uint8_t)(v & 0x00FFu);
  buf[1] = (uint8_t)((v >> 8) & 0x00FFu);
}

void UART_PutU32LE(uint8_t *buf, uint32_t v)
{
  if (buf == NULL)
  {
    return;
  }
  buf[0] = (uint8_t)(v & 0x000000FFuL);
  buf[1] = (uint8_t)((v >> 8) & 0x000000FFuL);
  buf[2] = (uint8_t)((v >> 16) & 0x000000FFuL);
  buf[3] = (uint8_t)((v >> 24) & 0x000000FFuL);
}

void UART_PutFloatLE(uint8_t *buf, float v)
{
  union
  {
    float    f;
    uint32_t u;
  } cvt;

  if (buf == NULL)
  {
    return;
  }

  cvt.f = v;
  UART_PutU32LE(buf, cvt.u);
}

/** 规范的 NaN 位型 (0x7FC00000): 上位机用它判断"该通道无效" */
float UART_NaN(void)
{
  union
  {
    float    f;
    uint32_t u;
  } cvt;

  cvt.u = UART_TEMP_NAN_BITS;
  return cvt.f;
}

/*==============================================================================
 * 发送环形缓冲
 *============================================================================*/
static uint16_t UART_TxFree(void)
{
  uint16_t head = s_tx_head;
  uint16_t tail = s_tx_tail;

  /* 留 1 字节区分"空"和"满" */
  if (tail > head)
  {
    return (uint16_t)(tail - head - 1u);
  }
  return (uint16_t)(UART_TX_RING_SIZE - head + tail - 1u);
}

static void UART_TxKick(void)
{
  uint16_t n;

  if ((s_tx_busy != 0u) || (s_tx_head == s_tx_tail))
  {
    return;
  }

  /* 只发"从 tail 到缓冲区末尾或到 head"这一段, 剩下的一段由 TxCplt 回调接着发 */
  if (s_tx_head > s_tx_tail)
  {
    n = (uint16_t)(s_tx_head - s_tx_tail);
  }
  else
  {
    n = (uint16_t)(UART_TX_RING_SIZE - s_tx_tail);
  }

  s_tx_chunk = n;
  s_tx_busy  = 1u;

  if (HAL_UART_Transmit_IT(&huart1, &s_tx_ring[s_tx_tail], n) != HAL_OK)
  {
    s_tx_busy = 0u;      /* 启动失败: 下一次 Poll/回调 再试 */
  }
}

uint8_t UART_Protocol_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  uint16_t total;
  uint16_t crc;
  uint8_t  i;
  uint32_t primask;

  if ((len > UART_DATA_MAX_LEN) || ((len > 0u) && (data == NULL)))
  {
    return 0u;
  }

  total = (uint16_t)len + 5u;   /* HEAD + CMD + LEN + DATA + CRC_LO + CRC_HI */

  /* 拷贝期间关中断: 否则 TxCplt 回调可能在我们写到一半时就把这段发出去。
     用 PRIMASK 保存/恢复, 保证"本来就已经关中断"的调用者不被打乱。 */
  primask = __get_PRIMASK();
  __disable_irq();

  if (UART_TxFree() < total)
  {
    __set_PRIMASK(primask);
    s_tx_overflow++;
    return 0u;
  }

  crc = 0xFFFFu;

  /* HEAD */
  crc = UART_CrcUpdate(crc, UART_FRAME_HEAD_UP);
  s_tx_ring[s_tx_head] = UART_FRAME_HEAD_UP;
  s_tx_head = (uint16_t)((s_tx_head + 1u) % UART_TX_RING_SIZE);

  /* CMD */
  crc = UART_CrcUpdate(crc, cmd);
  s_tx_ring[s_tx_head] = cmd;
  s_tx_head = (uint16_t)((s_tx_head + 1u) % UART_TX_RING_SIZE);

  /* LEN */
  crc = UART_CrcUpdate(crc, len);
  s_tx_ring[s_tx_head] = len;
  s_tx_head = (uint16_t)((s_tx_head + 1u) % UART_TX_RING_SIZE);

  /* DATA */
  for (i = 0u; i < len; i++)
  {
    crc = UART_CrcUpdate(crc, data[i]);
    s_tx_ring[s_tx_head] = data[i];
    s_tx_head = (uint16_t)((s_tx_head + 1u) % UART_TX_RING_SIZE);
  }

  /* CRC: 低字节在前 (MODBUS 习惯) */
  s_tx_ring[s_tx_head] = (uint8_t)(crc & 0x00FFu);
  s_tx_head = (uint16_t)((s_tx_head + 1u) % UART_TX_RING_SIZE);

  s_tx_ring[s_tx_head] = (uint8_t)((crc >> 8) & 0x00FFu);
  s_tx_head = (uint16_t)((s_tx_head + 1u) % UART_TX_RING_SIZE);

  __set_PRIMASK(primask);

  UART_TxKick();
  return 1u;
}

uint8_t UART_Protocol_SendTempFrame(const float temps[UART_TEMP_SLOT_COUNT])
{
  /* static: 128 字节, 不放栈上 (启动文件默认只有 1KB 栈) */
  static uint8_t buf[UART_TEMP_DATA_LEN];
  uint16_t i;

  if (temps == NULL)
  {
    return 0u;
  }

  /* ★ 固定 32 槽: 与板子通道数无关。本板没接的槽位由 main.c 预先填成 NaN,
     这里原样打包发走, 上位机看到 NaN 就知道"该通道不存在/断线"。 */
  for (i = 0u; i < UART_TEMP_SLOT_COUNT; i++)
  {
    UART_PutFloatLE(&buf[i * 4u], temps[i]);
  }

  return UART_Protocol_SendFrame(UART_CMD_UP_TEMP, buf, (uint8_t)UART_TEMP_DATA_LEN);
}

uint8_t UART_Protocol_SendStatusFrame(const UART_Status_t *st)
{
  static uint8_t d[UART_STATUS_DATA_LEN];

  if (st == NULL)
  {
    return 0u;
  }

  d[UART_ST_RUN_STATE] = st->run_state;
  d[UART_ST_DR_BITS]   = st->dr_bits;
  /* ★ byte2: 高 4 位 = 50/60 抑制(老格式不变), 低 4 位 = 本板实际片数。
     老上位机只取高 4 位, 因此完全兼容; 新上位机靠低 4 位知道"这片板子有几路"。 */
  d[UART_ST_REJECT]    = (uint8_t)((st->reject & 0x30u) | (st->chip_count & 0x0Fu));

  UART_PutU16LE(&d[UART_ST_CHIP_OK_LO],  st->chip_ok_mask);
  UART_PutU16LE(&d[UART_ST_CHIP_ERR_LO], st->chip_err_mask);
  UART_PutU32LE(&d[UART_ST_ROUND_LO],    st->round_count);
  UART_PutU32LE(&d[UART_ST_UPTIME_LO],   st->uptime_ms);
  UART_PutU16LE(&d[UART_ST_DRDY_CNT_LO], st->drdy_irq_count);
  UART_PutU16LE(&d[UART_ST_SPI_ERR_LO],  st->spi_err_count);
  UART_PutU16LE(&d[UART_ST_OPEN_CNT_LO], st->open_tc_count);

  d[UART_ST_FLAGS]     = st->flags;
  UART_PutU16LE(&d[UART_ST_TIMEOUT_LO], st->phase_timeout_count);

  return UART_Protocol_SendFrame(UART_CMD_UP_STATUS, d, UART_STATUS_DATA_LEN);
}

uint8_t UART_Protocol_SendRawFrame(const float emf_uv[UART_TEMP_SLOT_COUNT],
                                   const float cj_c[UART_CJ_SLOT_COUNT])
{
  /* static: 192 字节, 同样不放栈上 (启动文件默认只有 1KB 栈)。
     和温度帧用不同的缓冲区, 所以 TC_UPLINK_MODE=2(两帧都发) 时不会互相踩。 */
  static uint8_t buf[UART_RAW_DATA_LEN];
  uint16_t i;

  if ((emf_uv == NULL) || (cj_c == NULL))
  {
    return 0u;
  }

  /* 前半段: 32 槽热电势 (µV)。上位机拿它 + 自己的分度表算温度。 */
  for (i = 0u; i < UART_TEMP_SLOT_COUNT; i++)
  {
    UART_PutFloatLE(&buf[i * 4u], emf_uv[i]);
  }

  /* 后半段: 16 槽冷端温度 (°C)。冷端补偿必须用同一个算法,
     所以这两段必须一起发, 只给一半上位机算不出温度。 */
  for (i = 0u; i < UART_CJ_SLOT_COUNT; i++)
  {
    UART_PutFloatLE(&buf[UART_RAW_CJ_OFFSET + (i * 4u)], cj_c[i]);
  }

  return UART_Protocol_SendFrame(UART_CMD_UP_RAW, buf, (uint8_t)UART_RAW_DATA_LEN);
}

/*==============================================================================
 * 事件队列
 *============================================================================*/
static void UART_PushEvent(const UART_EventMsg_t *msg)
{
  uint8_t next = (uint8_t)((s_ev_head + 1u) % UART_EV_QUEUE_LEN);

  if (next == s_ev_tail)
  {
    return;   /* 队列满: 丢弃最新事件 (命令很少, 实际不会发生) */
  }

  s_ev_queue[s_ev_head] = *msg;
  s_ev_head = next;
}

uint8_t UART_Protocol_GetEvent(UART_EventMsg_t *msg)
{
  if ((msg == NULL) || (s_ev_tail == s_ev_head))
  {
    return 0u;
  }

  *msg = s_ev_queue[s_ev_tail];
  s_ev_tail = (uint8_t)((s_ev_tail + 1u) % UART_EV_QUEUE_LEN);

  return 1u;
}

/*==============================================================================
 * 下行命令解析
 *============================================================================*/
/** 解析 CMD=0x01 的采样率参数。
 *  LEN=2: DATA[0]=DR(索引0..6 或 已移位值), DATA[1]=50/60(索引0..3 或 已移位值)
 *  LEN=1: DATA[0]=(DR索引<<4)|(50/60索引)
 *  @retval 1 = 合法 */
static uint8_t UART_ParseRate(uint8_t len, const uint8_t *d,
                              uint8_t *dr_bits, uint8_t *reject)
{
  static const uint8_t dr_tbl[7] = { 0x00u, 0x20u, 0x40u, 0x60u, 0x80u, 0xA0u, 0xC0u };
  static const uint8_t rj_tbl[4] = { 0x00u, 0x10u, 0x20u, 0x30u };
  uint8_t dr;
  uint8_t rj;

  if (len == 1u)
  {
    dr = (uint8_t)((d[0] >> 4) & 0x0Fu);
    rj = (uint8_t)(d[0] & 0x0Fu);
  }
  else if (len >= 2u)
  {
    dr = d[0];
    rj = d[1];

    /* 兼容"已移位"写法: 0x20/0x40/.../0xC0 -> 索引 1/2/.../6 */
    if (dr > 6u)
    {
      if ((dr & 0x1Fu) != 0u)
      {
        return 0u;
      }
      dr = (uint8_t)(dr >> 5);
    }

    if (rj > 3u)
    {
      if ((rj & 0x0Fu) != 0u)
      {
        return 0u;
      }
      rj = (uint8_t)(rj >> 4);
    }
  }
  else
  {
    return 0u;
  }

  if ((dr > 6u) || (rj > 3u))
  {
    return 0u;
  }

  *dr_bits = dr_tbl[dr];
  *reject  = rj_tbl[rj];

  return 1u;
}

static void UART_DispatchFrame(void)
{
  UART_EventMsg_t msg;
  uint8_t  cmd = s_rx_buf[1];
  uint8_t  len = s_rx_buf[2];
  uint8_t *d   = &s_rx_buf[3];

  msg.dr_bits = 0u;
  msg.reject  = 0u;
  msg.channel = UART_CH_SINGLE_ALL;

  switch (cmd)
  {
    case UART_CMD_DOWN_SET_RATE:
      if (UART_ParseRate(len, d, &msg.dr_bits, &msg.reject) != 0u)
      {
        msg.ev = UART_EV_SET_RATE;
        UART_PushEvent(&msg);
      }
      break;

    case UART_CMD_DOWN_RUN:
      if (len >= 1u)
      {
        msg.ev = (d[0] != 0u) ? UART_EV_START : UART_EV_STOP;
        UART_PushEvent(&msg);
      }
      break;

    case UART_CMD_DOWN_SINGLE:
      msg.ev      = UART_EV_SINGLE;
      msg.channel = (len >= 1u) ? d[0] : UART_CH_SINGLE_ALL;
      UART_PushEvent(&msg);
      break;

    default:
      /* 未知命令: 忽略 (不回复, 便于以后扩展) */
      break;
  }
}

/*==============================================================================
 * 收帧状态机 (在串口中断里被逐字节调用)
 *============================================================================*/
void UART_Protocol_OnRxByte(uint8_t byte)
{
  s_rx_tick = HAL_GetTick();

  /* 兜底: 索引越界立刻回到找帧头, 避免缓冲区被写穿 */
  if (s_rx_idx >= UART_FRAME_MAX_LEN)
  {
    s_rx_state = RXS_HEAD;
    s_rx_idx   = 0u;
  }

  switch (s_rx_state)
  {
    case RXS_HEAD:
      if (byte == UART_FRAME_HEAD_DOWN)
      {
        s_rx_buf[0] = byte;
        s_rx_idx    = 1u;
        s_rx_state  = RXS_CMD;
      }
      /* 不是帧头就继续等 —— 天然的重新同步 */
      break;

    case RXS_CMD:
      s_rx_buf[s_rx_idx] = byte;
      s_rx_idx++;
      s_rx_state = RXS_LEN;
      break;

    case RXS_LEN:
      if (byte > UART_DATA_MAX_LEN)
      {
        s_rx_state = RXS_HEAD;      /* 长度非法: 重新找帧头 */
        s_rx_idx   = 0u;
        break;
      }
      s_rx_buf[s_rx_idx] = byte;
      s_rx_idx++;
      s_rx_need  = (uint16_t)byte;
      s_rx_state = (byte == 0u) ? RXS_CRC_LO : RXS_DATA;
      break;

    case RXS_DATA:
      s_rx_buf[s_rx_idx] = byte;
      s_rx_idx++;
      s_rx_need--;
      if (s_rx_need == 0u)
      {
        s_rx_state = RXS_CRC_LO;
      }
      break;

    case RXS_CRC_LO:
      s_rx_buf[s_rx_idx] = byte;
      s_rx_idx++;
      s_rx_state = RXS_CRC_HI;
      break;

    case RXS_CRC_HI:
      {
        uint16_t crc_calc;
        uint16_t crc_recv;

        s_rx_buf[s_rx_idx] = byte;
        s_rx_idx++;

        crc_calc = UART_CRC16(s_rx_buf, (uint16_t)(s_rx_idx - 2u));
        crc_recv = (uint16_t)s_rx_buf[s_rx_idx - 2u] |
                   ((uint16_t)s_rx_buf[s_rx_idx - 1u] << 8);

        if (crc_calc == crc_recv)
        {
          UART_DispatchFrame();
        }
        else
        {
          s_crc_err_cnt++;
        }

        s_rx_state = RXS_HEAD;
        s_rx_idx   = 0u;
      }
      break;

    default:
      s_rx_state = RXS_HEAD;
      s_rx_idx   = 0u;
      break;
  }
}

/*==============================================================================
 * 中断回调
 *============================================================================*/
static void UART_RxArm(void)
{
  if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u) == HAL_OK)
  {
    s_rx_armed = 1u;
  }
  else
  {
    s_rx_armed = 0u;    /* 挂不上就交给 UART_Protocol_Poll() 重试 */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  s_rx_armed = 0u;

  UART_Protocol_OnRxByte(s_rx_byte);

  UART_RxArm();       /* 立刻重新挂上, 不然只收到 1 个字节 */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  s_tx_tail = (uint16_t)((uint32_t)(s_tx_tail + s_tx_chunk) % UART_TX_RING_SIZE);
  s_tx_busy = 0u;

  UART_TxKick();      /* 环形缓冲里还有就接着发 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  /* 溢出/帧错/噪声都会让接收停摆, 清标志后重新挂中断 */
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  s_rx_armed = 0u;
  UART_RxArm();
}

/*==============================================================================
 * 初始化 / 轮询
 *============================================================================*/
void UART_Protocol_Init(void)
{
  s_rx_state  = RXS_HEAD;
  s_rx_idx    = 0u;
  s_rx_need   = 0u;
  s_rx_tick   = HAL_GetTick();
  s_rx_armed  = 0u;

  s_ev_head   = 0u;
  s_ev_tail   = 0u;

  s_tx_head   = 0u;
  s_tx_tail   = 0u;
  s_tx_chunk  = 0u;
  s_tx_busy   = 0u;
  s_tx_overflow = 0u;

  s_crc_err_cnt = 0u;

  UART_RxArm();
}

void UART_Protocol_Poll(void)
{
  /* 1) 帧内超时: 半截帧丢弃, 重新找帧头 */
  if ((s_rx_state != RXS_HEAD) &&
      ((HAL_GetTick() - s_rx_tick) > UART_FRAME_GAP_MS))
  {
    s_rx_state = RXS_HEAD;
    s_rx_idx   = 0u;
    s_rx_need  = 0u;
  }

  /* 2) 接收中断自愈 */
  if (s_rx_armed == 0u)
  {
    UART_RxArm();
  }

  /* 3) 发送续传 (正常情况下 TxCplt 已经处理, 这里只是兜底) */
  if ((s_tx_busy == 0u) && (s_tx_head != s_tx_tail))
  {
    UART_TxKick();
  }
}

uint16_t UART_Protocol_GetCrcErrCount(void)
{
  return s_crc_err_cnt;
}

uint16_t UART_Protocol_GetTxOverflowCount(void)
{
  return s_tx_overflow;
}

/*==============================================================================
 * 自检
 *============================================================================*/
uint8_t UART_Protocol_SelfTest(void)
{
  UART_EventMsg_t msg;
  uint8_t  frame[8];
  uint16_t crc;
  uint8_t  i;

  /* ---- 1) CRC-16/MODBUS 标准向量 ---- */
  if (UART_CRC16((const uint8_t *)"123456789", 9u) != 0x4B37u)
  {
    return 0u;
  }

  /* ---- 2) 组帧/拆帧闭环: 手工造一条下行"启动采集"命令喂给解析器 ---- */
  frame[0] = UART_FRAME_HEAD_DOWN;
  frame[1] = UART_CMD_DOWN_RUN;
  frame[2] = 1u;
  frame[3] = 1u;                          /* DATA[0] = 1 -> 启动 */
  crc = UART_CRC16(frame, 4u);
  frame[4] = (uint8_t)(crc & 0x00FFu);    /* CRC 低字节在前 */
  frame[5] = (uint8_t)((crc >> 8) & 0x00FFu);

  s_rx_state = RXS_HEAD;
  s_rx_idx   = 0u;

  for (i = 0u; i < 6u; i++)
  {
    UART_Protocol_OnRxByte(frame[i]);
  }

  if (UART_Protocol_GetEvent(&msg) == 0u)
  {
    return 0u;
  }
  if (msg.ev != UART_EV_START)
  {
    return 0u;
  }

  /* ---- 3) 小端打包 ---- */
  {
    uint8_t t[4];
    UART_PutU32LE(t, 0x12345678uL);
    if ((t[0] != 0x78u) || (t[1] != 0x56u) || (t[2] != 0x34u) || (t[3] != 0x12u))
    {
      return 0u;
    }
  }

  return 1u;
}
