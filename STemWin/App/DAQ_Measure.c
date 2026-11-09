/*
******************************************************************************
  文件        : DAQ_Measure.c
  功能        : 对 Remote DAQ 可见窗口内的信号做测量。
                模拟通道返回峰峰值、中心电平和频率；数字通道返回由上升沿
                得到的频率以及第一个高电平脉冲的宽度。周期按首个边沿到
                最后一个边沿计算，比统计屏幕上的整周期数稳定得多。

  说明        : 只读取采样环形缓冲，不写入任何内容，
                因此测量可以在绘制循环中执行。
  平台        : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#include <stdint.h>
#include "RemoteDAQ_Private.h"

/******************************************************************************
*  测量
******************************************************************************/

/* 在可见窗口内测量一路模拟通道：Vpp = 窗口内最大值 - 最小值，
   中心电平 = (max + min) / 2，并给出频率。穿越检测的迟滞由幅度导出，
   因此小信号中电平附近的噪声不会产生多余穿越。                              */
void App_MeasAnalog(int ch, int *pVpp, int *pFreq, int *pOff)
{
  int      i, idx, v, vMin, vMax, mid, hysteresis, state, crossings, freq;
  int      first, last;
  uint32_t sampleUs, spanUs, windowUs;

  idx  = App_StartIndex();
  vMin = 3300;
  vMax = 0;
  for (i = 0; i < N_DISP; i++)
  {
    v = (ch == 0) ? (int)aSamples[idx].ch1 : (int)aSamples[idx].ch2;
    if (v < vMin) vMin = v;
    if (v > vMax) vMax = v;
    idx++;
    if (idx >= App_BufLen()) idx = 0;
  }

  mid        = (vMin + vMax) / 2;
  hysteresis = (vMax - vMin) / 10;
  if (hysteresis < 20) hysteresis = 20;
  state      = 0;
  crossings  = 0;
  first      = -1;
  last       = -1;
  idx        = App_StartIndex();
  for (i = 0; i < N_DISP; i++)
  {
    v = (ch == 0) ? (int)aSamples[idx].ch1 : (int)aSamples[idx].ch2;
    if (state == 0)
    {
      if (v > (mid + hysteresis))
      {
        state = 1;
        crossings++;
        if (first < 0) first = i;
        last = i;
      }
    }
    else
    {
      if (v < (mid - hysteresis))
      {
        state = 0;
      }
    }
    idx++;
    if (idx >= App_BufLen()) idx = 0;
  }

  sampleUs = aTbUs[timebaseIdx] / SPP;   /* 两个采样点之间的时间，单位 us */
  windowUs = aTbUs[timebaseIdx] * 8u;

  /* 在固定窗口内统计穿越数会把结果量化到整周期：屏幕上只有 3 个周期时，
     多一次穿越就能把 375 Hz 变成 500 Hz。改为按首个穿越到最后一个穿越
     计算，得到的是同一窗口内的平均周期，稳定得多，也不受触发位置影响。   */
  if ((crossings >= 2) && (last > first))
  {
    spanUs = (uint32_t)(last - first) * sampleUs;
    freq   = (int)(((uint32_t)(crossings - 1) * 1000000u + spanUs / 2u) / spanUs);
  }
  else
  {
    freq   = (int)(((uint32_t)crossings * 1000000u) / windowUs);
  }

  *pVpp  = vMax - vMin;
  *pFreq = freq;
  *pOff  = mid;
}

/* 数字通道做同样的测量：由上升沿得到频率，
   并给出窗口中第一个高电平脉冲的宽度。                                       */
void App_MeasDigital(int ch, int *pFreq, int *pPw)
{
  int      i, idx, bitVal, state, edges, run, found;
  int      first, last;
  uint32_t sampleUs, spanUs, windowUs;

  sampleUs = aTbUs[timebaseIdx] / SPP;
  windowUs = aTbUs[timebaseIdx] * 8u;

  idx   = App_StartIndex();
  state = 0;
  edges = 0;
  run   = 0;
  found = 0;
  first = -1;
  last  = -1;

  for (i = 0; i < N_DISP; i++)
  {
    bitVal = (int)((aSamples[idx].dig >> ch) & 1u);
    if (bitVal)
    {
      if (state == 0)
      {
        state = 1;
        edges++;
        if (first < 0) first = i;
        last = i;
        run = 1;
      }
      else
      {
        run++;
      }
    }
    else
    {
      if (state == 1)
      {
        if (!found)
        {
          *pPw = (int)(run * sampleUs);
          found = 1;
        }
      }
      state = 0;
      run   = 0;
    }
    idx++;
    if (idx >= App_BufLen()) idx = 0;
  }

  if (!found)
  {
    *pPw = 0;
  }

  /* 与模拟通道思路相同：周期取首个边沿到最后一个边沿 */
  if ((edges >= 2) && (last > first))
  {
    spanUs  = (uint32_t)(last - first) * sampleUs;
    *pFreq  = (int)(((uint32_t)(edges - 1) * 1000000u + spanUs / 2u) / spanUs);
  }
  else
  {
    *pFreq  = (int)(((uint32_t)edges * 1000000u) / windowUs);
  }
}

/*************************** 文件结束 ****************************/
