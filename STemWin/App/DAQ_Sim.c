/*
******************************************************************************
  文件        : DAQ_Sim.c
  功能        : Remote DAQ 显示端的采样环形缓冲。
                两个数据源写入同一个环形缓冲：USART6 收到的帧由
                App_LoadFrame 追加（DAQ_Link.c），而在没有有效帧到达时，
                内置仿真器自行产生采样点，屏幕因此不会一直空白。两种模式
                因此共用一段连续记录，波形可以跨帧边界绘制而不出现接缝。

  还包括      : 选取屏幕窗口首个采样点的触发搜索，
                以及绘制和测量模块使用的小型访问函数。
  平台        : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#include <stdint.h>
#include "GUI.h"
#include "DAQ_Link.h"
#include "RemoteDAQ_Private.h"

/******************************************************************************
*  仿真（仅在未收到实时帧时使用）
******************************************************************************/

/* 四分之一周期正弦表，单位为千分之一。整周期通过在其他三个象限镜像得到，
   绘制循环里因此只需几次比较，而不必调用浮点正弦函数。                      */
static int App_Sine(uint32_t phase)
{
  static const int16_t aQ[17] = {
    0, 98, 195, 290, 383, 471, 556, 634,
    707, 773, 832, 882, 924, 957, 981, 995, 1000
  };
  int idx, q, r, v;

  idx = (int)((phase >> 26) & 63u);
  q   = idx >> 4;
  r   = idx & 15;
  if (q == 0)      v = aQ[r];
  else if (q == 1) v = aQ[16 - r];
  else if (q == 2) v = -aQ[r];
  else             v = -aQ[16 - r];
  return v;
}

/* 向环形缓冲生成 n 个仿真采样点。两个相位累加器产生两路模拟信号，
   数字通道只是简单的时间比较，演示画面因此会自行滚动。                      */
void App_GenSamples(int n)
{
  int      k, v1, v2;
  uint32_t sampleUs;
  SCOPE_SAMPLE *pSample;

  sampleUs = aTbUs[timebaseIdx] / SPP;

  for (k = 0; k < n; k++)
  {
    v1 = App_Sine(simPhaseA);
    v2 = App_Sine(simPhaseB);

    pSample = &aSamples[sampleWriteIdx];
    pSample->ch1 = (uint16_t)(1500 + (v1 * 550) / 1000);
    pSample->ch2 = (uint16_t)(1500 + (v2 * 605) / 1000);

    pSample->dig = 0;
    if ((simTimeUs % 2000u) < 1000u) pSample->dig |= 1u;
    if ((simTimeUs % 1000u) < 500u)  pSample->dig |= 2u;
    if ((simTimeUs % 1333u) < 327u)  pSample->dig |= 4u;

    simTimeUs += sampleUs;
    simPhaseA += simStepA;
    simPhaseB += simStepB;

    sampleWriteIdx++;
    if (sampleWriteIdx >= N_SAMP)
    {
      sampleWriteIdx = 0;
    }
  }
}

/* 按当前选中的时基重启仿真器：相位步进就是音调频率换算成的采样点数，
   因此改变时基会改变同一信号在屏幕上的样子。                                */
void App_ResetBuffer(void)
{
  uint32_t sampleUs;

  sampleUs = aTbUs[timebaseIdx] / SPP;
  simStepA = (uint32_t)((((uint64_t)375u * sampleUs) << 32) / 1000000u);
  simStepB = (uint32_t)((((uint64_t)875u * sampleUs) << 32) / 1000000u);

  simTimeUs      = 0;
  simPhaseA      = 0;
  simPhaseB      = 0;
  sampleWriteIdx = 0;
  App_GenSamples(N_SAMP);
}

/******************************************************************************
*  真实帧 -> 显示缓冲
******************************************************************************/

/* 把采集板给出的时基映射为按键使用的本地索引，未知值返回 -1。              */
static int App_TbIndex(uint16_t us)
{
  int i;

  for (i = 0; i < 5; i++)
  {
    if (aTbUs[i] == us)
    {
      return i;
    }
  }
  return -1;
}

/* 每个收到的帧都追加到仿真器所用的同一个环形缓冲。连续帧于是构成一段连续
   记录，波形可以跨帧边界绘制而不出现接缝：此前每帧整体替换缓冲，正是
   波形跳变的原因。                                                          */
void App_LoadFrame(void)
{
  int i, tbi, writeIdx;
  uint16_t raw;
  uint8_t  dg;

  writeIdx = (int)sampleWriteIdx;
  for (i = 0; i < N_DISP; i++)
  {
    /* 链路上传输的是原始 ADC 码值，屏幕以毫伏为单位 */
    raw = gDaq.ch1[i];
    aSamples[writeIdx].ch1 = (uint16_t)(((uint32_t)raw * 3300u + 2047u) / 4095u);

    raw = gDaq.ch2[i];
    aSamples[writeIdx].ch2 = (uint16_t)(((uint32_t)raw * 3300u + 2047u) / 4095u);

    dg = 0;
    if (DAQ_DigGet(&gDaq.dig[0][0], 0, (uint16_t)i)) dg |= 1u;
    if (DAQ_DigGet(&gDaq.dig[0][0], 1, (uint16_t)i)) dg |= 2u;
    if (DAQ_DigGet(&gDaq.dig[0][0], 2, (uint16_t)i)) dg |= 4u;
    aSamples[writeIdx].dig = dg;

    writeIdx++;
    if (writeIdx >= N_SAMP) writeIdx = 0;
  }
  sampleWriteIdx = (uint16_t)writeIdx;

  tbi = App_TbIndex(gDaq.timebaseUs);
  if (tbi >= 0)
  {
    timebaseIdx = tbi;
  }

  liveMode      = 1;
  gDaq.newFrame = 0;
}

/* 某通道的显示开关，开关状态属于按键页面。 */
int App_Vis(int i)
{
  return channelOn[i];
}

/* 真实帧同样追加到环形缓冲，因此两种模式始终共用同一段连续记录 */
int App_BufLen(void)
{
  return N_SAMP;
}

/* 一个采样点的显示值：模拟通道为毫伏，数字通道为 0 / 1。                    */
static int App_SampleVal(int i, int ch)
{
  if (ch == 0) return (int)aSamples[i].ch1;
  if (ch == 1) return (int)aSamples[i].ch2;
  return (int)((aSamples[i].dig >> (ch - 2)) & 1u);
}

/* 屏幕窗口的首个采样点，按触发条件对齐。
   只在窗口能完整落在写指针之前的那些位置上搜索。若触发点更靠右，
   窗口会绕回最旧的采样点，时间轴会在屏幕中间倒退：那就是可见的接缝。
   电平判断带迟滞，当用户设定超出信号范围时退回波形中点。                    */
int App_StartIndex(void)
{
  int len, candidateCount, i, j, v, vMin, vMax, level, hysteresis, armed, prev, start, hit;

  len            = App_BufLen();
  candidateCount = len - N_DISP - 1;     /* 窗口始终结束于 sampleWriteIdx 之前 */
  start          = (int)sampleWriteIdx;  /* 最旧采样点，无触发时使用 */
  hit            = -1;

  if (triggerSource < 2)
  {
    /* 电平必须落在信号范围内，否则永远不会有穿越 */
    vMin = 3300;
    vMax = 0;
    for (i = 0; i <= candidateCount; i++)
    {
      j = (int)sampleWriteIdx + i;
      if (j >= len) j -= len;

      v = App_SampleVal(j, triggerSource);
      if (v < vMin) vMin = v;
      if (v > vMax) vMax = v;
    }

    hysteresis = (vMax - vMin) / 8;
    if (hysteresis < 20) hysteresis = 20;
    level = triggerLevelMv;
    if ((level <= (vMin + hysteresis)) || (level >= (vMax - hysteresis)))
    {
      /* 触发电平落在信号之外：自动模式退回中点，保证画面总有波形；
         正常 / 单次模式则当作没有触发，由主循环保持上一帧 */
      if (triggerMode != 0)
      {
        trigFound = 0;
        return start;
      }
      level = (vMin + vMax) / 2;
    }

    /* 信号低于 level - 迟滞时置位，高于 level 时触发。取记录中最后一次穿越，
       这样显示的是最近可能的窗口，而不是滞后整整一段记录。                   */
    armed = 0;
    for (i = 1; i <= candidateCount; i++)
    {
      j = (int)sampleWriteIdx + i;
      if (j >= len) j -= len;

      v = App_SampleVal(j, triggerSource);
      if (armed == 0)
      {
        if (v < (level - hysteresis))
        {
          armed = 1;
        }
      }
      else if (v >= level)
      {
        armed = 0;
        hit   = j;
      }
    }
  }
  else
  {
    /* 数字源：最后一个上升沿 */
    prev = App_SampleVal((int)sampleWriteIdx, triggerSource);
    for (i = 1; i <= candidateCount; i++)
    {
      j = (int)sampleWriteIdx + i;
      if (j >= len) j -= len;

      v = App_SampleVal(j, triggerSource);
      if ((prev == 0) && (v != 0))
      {
        hit = j;
      }
      prev = v;
    }
  }

  trigFound = (hit >= 0) ? 1 : 0;
  if (hit < 0)
  {
    return start;                        /* 没找到触发点：显示最旧的一段 */
  }

  /* 触发点放在窗口左侧 1/5 处，触发沿前后的波形都能看到 */
  hit -= TRIG_POS;
  if (hit < 0)
  {
    hit += len;
  }
  return hit;
}

/*************************** 文件结束 ****************************/
