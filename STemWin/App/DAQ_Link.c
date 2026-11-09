/*
******************************************************************************
  File        : DAQ_Link.c
  Purpose     : USART6 链路 - 接收、帧同步、CRC 校验、载荷解析
  Notes       : - USART6 用 PC6(TX) / PC7(RX)，即 Arduino D1 / D0 排针
                - 4000000 波特率（与采集端一致），8N1，无流控
                - 接收由 DMA2 Stream1 / 通道 5 循环搬运，所以没有逐字节中断，
                  接收端也不占 HAL 锁；帧状态机在主循环中运行，不在中断里
  Data flow   : USART6 -> SRAM1 中的 DMA 循环缓冲 -> rxRing -> 字节状态机
                -> frameBuf -> 版本 / 长度 / CRC 校验 -> gDaq
                应用只读 gDaq，因此半帧数据永远不会送到显示。
******************************************************************************
*/

#include <string.h>
#include "DAQ_Link.h"

#define RX_RING_SIZE   8192u          /* 2 的幂，约 4.6 帧                 */
#define RX_RING_MASK   (RX_RING_SIZE - 1u)

#define FRAME_VER      0x01u
#define FRAME_TYPE     0x01u

UART_HandleTypeDef huart6;
DMA_HandleTypeDef  hdmaUsart6Rx;
DAQ_FRAME          gDaq;

/* DMA 无法访问 DTCM(0x20000000~0x2000FFFF)，而 IAR 链接器通常把 .bss 放在那里，
   因此接收缓冲固定在 SRAM1 起始处。缓冲按 32 字节对齐，以满足 DAQ_Link_Poll()
   中 D-cache 维护的要求。若其它变量长到这块窗口内，链接器会报重叠错误。       */
#if defined ( __ICCARM__ )
#pragma location = 0x20010000
#endif
static uint8_t rxRing[RX_RING_SIZE];

static volatile uint16_t rxTail;

static uint8_t  frameBuf[DAQ_FRAME_SIZE];
static uint16_t framePos;
static uint8_t  syncStage;            /* 0 = 等 AA，1 = 等 55，2 = 收帧        */

/******************************************************************************
*  CRC-16/CCITT-FALSE：多项式 0x1021，初值 0xFFFF，不反射，结果异或 0
******************************************************************************/

/* 故意用逐位算法：查表版需要在 flash 里放整张表，而这里只在整帧校验时算一次，
   并非逐字节计算。参数与采集板所用的一致，即 CRC-16/CCITT-FALSE。             */
static uint16_t CRC16(const uint8_t *data, uint32_t len)
{
  uint16_t crc;
  uint32_t i;
  uint8_t  j;

  crc = 0xFFFFu;
  for (i = 0; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (j = 0; j < 8u; j++)
    {
      if (crc & 0x8000u)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

/* 小端 16 位读取，线上格式与发送端约定一致                              */
static uint16_t GetU16(const uint8_t *p)
{
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/******************************************************************************
*  载荷解析
******************************************************************************/

/* 把一帧已校验的数据拷入 gDaq。帧布局由 DAQ_Link.h 的协议说明规定：序号、
   模式、通道掩码、时基、PWM 参数，然后是两段模拟数据，最后是打包的数字位。
   序号用于统计链路丢了多少帧；所有字段都拷完之后才置起 newFrame，所以显示侧
   看到的始终是一条完整记录。                                                 */
static void DAQ_ParseFrame(void)
{
  uint32_t seq, gap;
  uint16_t idx;

  seq = (uint32_t)frameBuf[4]
      | ((uint32_t)frameBuf[5] << 8)
      | ((uint32_t)frameBuf[6] << 16)
      | ((uint32_t)frameBuf[7] << 24);

  if (gDaq.valid)
  {
    gap = seq - gDaq.seq - 1u;    /* 无符号运算自动处理回绕            */
    if (gap < 0x80000000u)
    {
      gDaq.lost += gap;
    }
  }

  gDaq.mode       = frameBuf[10];
  gDaq.chMask     = frameBuf[11];
  gDaq.timebaseUs = GetU16(&frameBuf[12]);
  gDaq.pwmFreq    = GetU16(&frameBuf[14]);
  gDaq.pwmDuty    = frameBuf[16];
  gDaq.seq        = seq;

  /* 两段模拟数据各含 DAQ_POINTS 个 16 位采样：CH1 从偏移 18 开始，CH2 从 818
     开始，打包的数字位从 1618 开始。CRC 在 1768，紧随载荷之后。              */
  for (idx = 0; idx < DAQ_POINTS; idx++)
  {
    gDaq.ch1[idx] = GetU16(&frameBuf[18 + idx * 2]);
    gDaq.ch2[idx] = GetU16(&frameBuf[818 + idx * 2]);
  }
  memcpy(gDaq.dig, &frameBuf[1618], 150);

  gDaq.frames++;
  gDaq.lastTick = HAL_GetTick();
  gDaq.valid    = 1;
  gDaq.newFrame = 1;
}

/* 对候选帧做合理性检查：帧头版本、声明长度和 CRC。任何一项不通过都计一次错误
   并丢弃，解析器随后在下一个帧头字节处重新同步。                             */
static void DAQ_CheckFrame(void)
{
  uint16_t len, crc, calc;

  if ((frameBuf[2] != FRAME_VER) || (frameBuf[3] != FRAME_TYPE))
  {
    gDaq.errors++;
    return;
  }

  len = GetU16(&frameBuf[8]);
  if (len != DAQ_PAYLOAD_SIZE)
  {
    gDaq.errors++;
    return;
  }

  crc  = GetU16(&frameBuf[1768]);
  calc = CRC16(&frameBuf[2], 1766u);       /* 覆盖偏移 2 .. 1767 */
  if (crc != calc)
  {
    gDaq.errors++;
    return;
  }

  DAQ_ParseFrame();
}

/******************************************************************************
*  字节流状态机
******************************************************************************/

/* 把一个接收字节送入帧同步器：先找两个帧头字节，再收集完整一帧；收完立即
   重新搜索下一个帧头，因此坏帧的代价只有这一帧。                             */
static void DAQ_FeedByte(uint8_t b)
{
  switch (syncStage)
  {
    case 0:                              /* 等待 AA          */
      if (b == 0xAAu)
      {
        syncStage = 1;
      }
      break;

    case 1:                              /* 已收到 AA，等待 55     */
      if (b == 0x55u)
      {
        frameBuf[0] = 0xAAu;
        frameBuf[1] = 0x55u;
        framePos    = 2;
        syncStage   = 2;
      }
      else if (b != 0xAAu)               /* AA AA 0x55 仍保持同步    */
      {
        syncStage = 0;
      }
      break;

    default:                             /* 收集帧的剩余部分                 */
      frameBuf[framePos++] = b;
      if (framePos >= DAQ_FRAME_SIZE)
      {
        syncStage  = 0;
        framePos   = 0;
        DAQ_CheckFrame();
      }
      break;
  }
}

/******************************************************************************
*  物理回环检测：D1 (PC6) 作普通 GPIO 输出，D0 (PC7) 作输入采样。该检测在
*  USART 接管这两个引脚之前执行，因此能判断两个排针是否真的用线相连，与
*  USART 配置无关。
******************************************************************************/

/* 先把 PC6 拉低再拉高，同时回读 PC7，这样不用跑任何协议就能区分“线没接”
   和“波特率不对”。                                                         */
static uint8_t DAQ_PhysLoop(void)
{
  GPIO_InitTypeDef gpio;
  volatile int     spin;
  uint8_t          lowOk, highOk;

  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Pin       = GPIO_PIN_6;
  gpio.Mode      = GPIO_MODE_OUTPUT_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin       = GPIO_PIN_7;
  gpio.Mode      = GPIO_MODE_INPUT;
  gpio.Pull      = GPIO_PULLDOWN;
  gpio.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &gpio);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  for (spin = 0; spin < 200; spin++) { }
  lowOk = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET) ? 1u : 0u;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  for (spin = 0; spin < 200; spin++) { }
  highOk = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) ? 1u : 0u;

  return (uint8_t)(lowOk & highOk);
}

/******************************************************************************
*  对外接口
******************************************************************************/

/* 初始化采集链路：先做物理回环检测，再把两个引脚配成 USART6 和循环 DMA，
   DMA 会自动填充 rxRing。                                                   */
void DAQ_Link_Init(void)
{
  GPIO_InitTypeDef gpio;

  rxTail    = 0;
  syncStage = 0;
  framePos  = 0;
  memset(&gDaq, 0, sizeof(gDaq));

  /* 必须在 USART 接管引脚之前完成             */
  gDaq.physLoop = DAQ_PhysLoop();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();

  /* PC6 = USART6_TX，PC7 = USART6_RX（Arduino D1 / D0） */
  gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &gpio);

  huart6.Instance          = USART6;
  huart6.Init.BaudRate     = 4000000;
  huart6.Init.WordLength   = UART_WORDLENGTH_8B;
  huart6.Init.StopBits     = UART_STOPBITS_1;
  huart6.Init.Parity       = UART_PARITY_NONE;
  huart6.Init.Mode         = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;  /* PCLK2 = 100 MHz */
  HAL_UART_Init(&huart6);

  /* USART6_RX 用 DMA2 Stream1 / 通道 5，循环模式：DMA 自动搬运每个字节到
     rxRing 并自动回绕，不产生逐字节中断。                                   */
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdmaUsart6Rx.Instance                 = DMA2_Stream1;
  hdmaUsart6Rx.Init.Channel             = DMA_CHANNEL_5;
  hdmaUsart6Rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdmaUsart6Rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdmaUsart6Rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdmaUsart6Rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdmaUsart6Rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdmaUsart6Rx.Init.Mode                = DMA_CIRCULAR;
  hdmaUsart6Rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdmaUsart6Rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdmaUsart6Rx.Init.FIFOThreshold        = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  hdmaUsart6Rx.Init.MemBurst            = DMA_MBURST_SINGLE;
  hdmaUsart6Rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&hdmaUsart6Rx);

  __HAL_LINKDMA(&huart6, hdmarx, hdmaUsart6Rx);

  HAL_NVIC_SetPriority(USART6_IRQn, 1, 0);          /* 帧错误 / 溢出          */
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);     /* 传输完成               */
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  HAL_UART_Receive_DMA(&huart6, rxRing, RX_RING_SIZE);
}

/* 把上次调用之后 DMA 新存入 rxRing 的字节交给帧状态机。不阻塞、不等整帧，
   只把当前可用的数据取完。                                                   */
void DAQ_Link_Poll(void)
{
  uint16_t headPos;
  uint16_t avail;

  /* 兜底：若错误中止了接收、而错误回调没能重启，就在这里恢复链路                                                                                        */
  if (huart6.RxState != HAL_UART_STATE_BUSY_RX)
  {
    rxTail = 0;
    HAL_UART_Receive_DMA(&huart6, rxRing, RX_RING_SIZE);
    return;                            /* DMA 从下标 0 重新开始               */
  }

  /* DMA 是环形写入：已写入字节数 = 缓冲大小 - 剩余传输计数                                                                                             */
  headPos = (uint16_t)(RX_RING_SIZE - __HAL_DMA_GET_COUNTER(&hdmaUsart6Rx));
  if (headPos >= RX_RING_SIZE)         /* 刚绕完一圈，计数器已重装             */
  {
    headPos = 0;
  }

  avail = (uint16_t)((headPos + RX_RING_SIZE - rxTail) & RX_RING_MASK);
  if (avail == 0u)
  {
    return;
  }

  /* DMA 写入的是 D-cache 背后的内存：读之前先让 CPU 可能缓存的行失效                                                                                        */
  SCB_InvalidateDCache_by_Addr((void *)rxRing, (int32_t)RX_RING_SIZE);

  while (avail--)
  {
    uint16_t tailIdx = rxTail;

    gDaq.rxBytes++;                   /* 诊断用：是否有数据到达              */
    DAQ_FeedByte(rxRing[tailIdx]);
    rxTail = (uint16_t)((tailIdx + 1u) & RX_RING_MASK);
  }
}

/* 取一个打包的数字采样：每通道 DAQ_POINTS 位、每字节 8 位，所以由采样序号
   算出字节下标和字节内的位号。                                               */
uint8_t DAQ_DigGet(const uint8_t *dig, uint8_t ch, uint16_t i)
{
  return (uint8_t)((dig[ch * DAQ_DIG_BYTES + (i >> 3)] >> (i & 7u)) & 1u);
}

/* HAL_UART_Transmit() 在整个发送期间持有 HAL 锁，RX 中断回调因此无法重新
   武装接收（返回 HAL_BUSY），逐字节中断保持关闭：链路收到一个字节后就哑了。
   这里直接操作外设寄存器，不占锁，所以发送一帧时接收仍在继续。               */
static void UART_SendRaw(USART_TypeDef *inst, const uint8_t *buf, uint16_t len)
{
  uint16_t idx;

  for (idx = 0; idx < len; idx++)
  {
    while ((inst->ISR & USART_ISR_TXE) == 0u) { }
    inst->TDR = buf[idx];
  }
  while ((inst->ISR & USART_ISR_TC) == 0u) { }
}

/* ASCII 控制帧："A<mode>B<freq>C<duty>D"（mode 1..5）         */

/* 组装并发送该帧，同时在 gDaq.lastCtrl 留一份副本，界面可显示真正从引脚
   发出的最后一条命令。                                                       */
void DAQ_Link_SendControl(uint8_t mode, uint16_t freq, uint8_t duty)
{
  char     buf[32];
  char     digits[8];
  char    *out = buf;
  int      digitCount;
  uint16_t value;
  uint8_t  outLen;

  *out++ = 'A';
  *out++ = (char)('0' + (mode % 10u));
  *out++ = 'B';

  value = freq;
  digitCount = 0;
  do {
    digits[digitCount++] = (char)('0' + (value % 10u));
    value = (uint16_t)(value / 10u);
  } while (value);
  while (digitCount) *out++ = digits[--digitCount];

  *out++ = 'C';
  value = duty;
  digitCount = 0;
  do {
    digits[digitCount++] = (char)('0' + (value % 10u));
    value = (uint16_t)(value / 10u);
  } while (value);
  while (digitCount) *out++ = digits[--digitCount];

  *out++ = 'D';

  outLen = (uint8_t)(out - buf);
  memcpy(gDaq.lastCtrl, buf, (size_t)outLen);   /* 显示在状态栏            */
  gDaq.lastCtrl[outLen] = 0;
  UART_SendRaw(USART6, (const uint8_t *)buf, outLen);
  gDaq.txBytes += outLen;
}

/******************************************************************************
*  主机链路：ST-LINK 虚拟串口，仅用于导出 CSV / BMP 文件
*  USART1 TX = PA9 (AF7)，RX = PB7，921600 8N1
******************************************************************************/
UART_HandleTypeDef huart1;

static uint8_t hostRxBuf[256];
static volatile uint16_t hostRxHead;
static volatile uint16_t hostRxTail;
static uint8_t hostRxByte;

/* USART1 面向 PC 侧：每收到一个字符进一次中断就够，因为下载的数据量小，
   不需要流式传输。                                                           */
void Host_Init(void)
{
  GPIO_InitTypeDef gpio;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9 = USART1_TX，PB7 = USART1_RX：ST-LINK 虚拟串口          */
  gpio.Pin       = GPIO_PIN_9;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin       = GPIO_PIN_7;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &gpio);

  huart1.Instance          = USART1;
  /* ST-LINK 虚拟串口在 921600 以上无法工作，且该波特率必须与主机脚本
     download_daq_files.py 一致                                                */
  huart1.Init.BaudRate     = 921600;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

  hostRxHead = 0;
  hostRxTail = 0;

  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &hostRxByte, 1);
}

/* 取出一个已收到的字符，没有数据时返回 -1。本函数在主循环调用，中断只负责
   填充这个小环形缓冲。                                                       */
int Host_GetChar(void)
{
  uint16_t tail = hostRxTail;
  int      ch;

  if (tail == hostRxHead)
  {
    return -1;
  }
  ch = hostRxBuf[tail];
  hostRxTail = (uint16_t)((tail + 1u) & 0xFFu);
  return ch;
}

void Host_Put(const uint8_t *buf, uint16_t len)
{
  UART_SendRaw(USART1, buf, len);         /* 不占 HAL 锁，见 UART_SendRaw  */
}

/******************************************************************************
*  HAL 回调
******************************************************************************/

/* USART6 由循环 DMA 接收，传输完成回调无事可做；USART1 则需在每收到一个
   字符后重新武装单字节接收。                                                 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    /* 循环 DMA：传输自动重启，这里无需处理                               */
  }
  else if (huart->Instance == USART1)
  {
    uint16_t next = (uint16_t)((hostRxHead + 1u) & 0xFFu);

    if (next != hostRxTail)
    {
      hostRxBuf[hostRxHead] = hostRxByte;
      hostRxHead = next;
    }
    gDaq.hostRxBytes++;                 /* PC 侧有请求                       */
    HAL_UART_Receive_IT(&huart1, &hostRxByte, 1);
  }
}

/* 任何接收错误都会让 HAL 内部中止 DMA 流，所以必须在这里重新启动接收；解析器
   会在下一个帧头字节处重新同步，因此坏帧只影响这一帧。                       */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t blocking = huart->ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO);

  if (huart->Instance == USART6)
  {
    gDaq.uartErrors++;                 /* 波特率不匹配会在这里暴露           */

    /* DMA 模式下 HAL 遇到任何错误都会中止接收，因此要重新启动 DMA 流；
       解析器会在下一个 AA 55 处重新同步                                       */
    if (huart->RxState != HAL_UART_STATE_READY)
    {
      HAL_UART_AbortReceive(huart);    /* 清除 DMA 请求                      */
    }
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

    rxTail = 0;
    HAL_UART_Receive_DMA(&huart6, rxRing, RX_RING_SIZE);
  }
  else if (huart->Instance == USART1)
  {
    gDaq.uartErrors++;
    if (blocking)
    {
      HAL_UART_AbortReceive(huart);
      HAL_UART_Receive_IT(&huart1, &hostRxByte, 1);
    }
  }
}

/*************************** 文件结束 ***************************   */
