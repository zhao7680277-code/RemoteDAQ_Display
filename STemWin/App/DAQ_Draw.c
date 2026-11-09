/*
******************************************************************************
  文件          : DAQ_Draw.c
  用途          : Remote DAQ 显示的绘制层。
                所有送到 LCD 的内容都在这里构建：顶栏、随页面变化的按钮列、
                网格、五条波形、测量值行和链路状态条。整屏先在 memory
                device 中合成，再一次性拷贝，从而消除直接绘制时的闪烁。

  说明          : 本模块只读取应用状态（定义在 RemoteDAQ.c 中），从不修改它。
  平台          : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#include <stdint.h>
#include "GUI.h"
#include "DAQ_Link.h"
#include "RemoteDAQ_Private.h"

/******************************************************************************
*  绘制辅助函数
******************************************************************************/

/* 单个按钮：填充主体、边框和居中文字。 */
static void App_Button(int x, int y, int w, int h, const char *s, int bg, int fg)
{
  GUI_SetColor(bg);
  GUI_FillRect(x, y, x + w - 1, y + h - 1);
  GUI_SetColor(C_FRAME);
  GUI_DrawRect(x, y, x + w - 1, y + h - 1);
  GUI_SetColor(fg);
  GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_VCENTER);
  GUI_DispStringAt(s, x + w / 2, y + h / 2);
  GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);
}

/* 顶栏：标题、两个保存按钮、时间基准、链路计数和页码按钮。
   它每帧都会重绘，因此计数始终是最新的。 */
static void App_DrawTop(void)
{
  char ac[24];

  GUI_SetColor(C_BAR);
  GUI_FillRect(0, 0, SCR_W - 1, TOP_H - 1);

  GUI_SetColor(GUI_WHITE);
  GUI_DispStringAt("Remote DAQ", 4, 7);

  App_Button( 70, 3, 62, 16, "SAVE CSV", C_BTN, GUI_WHITE);
  App_Button(134, 3, 62, 16, "SAVE BMP", C_BTN, GUI_WHITE);

  /* 时间基准：同样可点击，在 5 个档位间循环 */
  ac[0] = 0;
  Str_App(ac, apTbName[timebaseIdx]);
  GUI_SetColor(GUI_WHITE);
  GUI_DispStringAt(ac, 202, 7);

  /* RUN 文字不变，暂停只是冻结波形 */
  GUI_SetColor(GUI_GREEN);
  GUI_DispStringAt("RUN", 260, 7);

  /* 上次保存的结果："CSV2 OK" / "BMP2 OK"，始终可见 */
  if (statusText[0])
  {
    GUI_SetColor(GUI_GREEN);
    GUI_DispStringAt(statusText, 284, 7);
  }

  /* 接收计数：快速确认数据是否真的到达。
     还没有有效帧？改为显示原始字节数，这样“链路上没有数据”和
     “有字节到达但帧未被识别”（波特率错误、帧头错误）的区别就能在屏幕上看到。 */
  ac[0] = 0;
  if (gDaq.frames)
  {
    Str_App(ac, "F");
    Str_Int(ac, (unsigned)gDaq.frames);
    Str_App(ac, " E");
    Str_Int(ac, (unsigned)gDaq.errors);
  }
  else
  {
    Str_App(ac, "B");
    Str_Int(ac, (unsigned)gDaq.rxBytes);
    Str_App(ac, " E");
    Str_Int(ac, (unsigned)gDaq.uartErrors);   /* UART 层错误，不是帧错误 */

    /* L1 = 上电时检测到 D1 (PC6) 与 D0 (PC7) 短接，
       L0 = 未短接。仅在链路仍然断开时有参考意义。 */
    Str_App(ac, " L");
    Str_Int(ac, (unsigned)gDaq.physLoop);
  }
  GUI_SetColor(GUI_WHITE);
  GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_TOP);
  GUI_DispStringAt(ac, 418, 7);
  GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);

  /* 页码按钮，位于右上角 */
  ac[0] = 0;
  Str_App(ac, "P");
  Str_Int(ac, (unsigned)page);
  App_Button(428, 3, 48, 16, ac, C_BTN2, GUI_WHITE);
}

/******************************************************************************
*  右侧按钮列：唯一随页面变化的部分
******************************************************************************/

/* 绘制当前页面对应的按钮列。所有页面的几何布局相同，
   只是按钮上的文字和上方显示的数值变化，
   因此触摸处理可以继续基于行号。 */
static void App_DrawSide(void)
{
  static const char *apName[8] = { "CH1", "CH2", "D0", "D1", "D2", "T-", "T+", "RUN" };
  char ac[12];
  int  i, y;

  /* 清空整列 */
  GUI_SetColor(GUI_BLACK);
  GUI_FillRect(SIDE_X, TOP_H, SCR_W - 1, SCR_H - 1);

  if (page == 1)
  {
    for (i = 0; i < 8; i++)
    {
      y = SIDE_Y + i * SIDE_STEP;

      if (i < 5)
      {
        if (channelOn[i])
        {
          App_Button(SIDE_X, y, SIDE_W, SIDE_H, apName[i], aChColor[i], aChFg[i]);
        }
        else
        {
          App_Button(SIDE_X, y, SIDE_W, SIDE_H, apName[i], C_BTN, C_OFF);
        }
      }
      else if (i == 7)
      {
        App_Button(SIDE_X, y, SIDE_W, SIDE_H, "RUN", GUI_GREEN, GUI_BLACK);
      }
      else
      {
        App_Button(SIDE_X, y, SIDE_W, SIDE_H, apName[i], C_BTN2, GUI_WHITE);
      }
    }
  }
  else if (page == 2)
  {
    y = SIDE_Y;

    ac[0] = 0;
    Str_Short(ac, pwmFreq);
    Str_App(ac, "Hz");
    GUI_SetColor(GUI_YELLOW);
    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_TOP);
    GUI_DispStringAt(ac, SIDE_X + SIDE_W / 2, y + 6);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "F-", C_BTN, GUI_WHITE);
    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "F+", C_BTN, GUI_WHITE);

    y += SIDE_STEP;
    ac[0] = 0;
    Str_Int(ac, (unsigned)pwmDuty);
    Str_App(ac, "%");
    GUI_SetColor(GUI_YELLOW);
    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_TOP);
    GUI_DispStringAt(ac, SIDE_X + SIDE_W / 2, y + 6);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "D-", C_BTN, GUI_WHITE);
    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "D+", C_BTN, GUI_WHITE);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "RUN", GUI_GREEN, GUI_BLACK);

    /* 垂直灵敏度，在 660 -> 33 mV/格 间循环 */
    y += SIDE_STEP;
    ac[0] = 0;
    Str_Int(ac, (unsigned)aVdiv[vIdx]);
    Str_App(ac, "mV");
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, ac, C_BTN2, GUI_YELLOW);
  }
  else if (page == 3)
  {
    static const char *apSrc[5] = { "CH1", "CH2", "D0", "D1", "D2" };

    y = SIDE_Y;
    ac[0] = 0;
    Str_App(ac, apTrig[triggerMode]);
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, ac, C_BTN, GUI_WHITE);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, apSrc[triggerSource], C_BTN2, GUI_WHITE);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "LVL-", C_BTN, GUI_WHITE);
    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "LVL+", C_BTN, GUI_WHITE);

    y += SIDE_STEP;
    ac[0] = 0;
    Str_Volt2(ac, triggerLevelMv);
    Str_App(ac, "V");
    GUI_SetColor(GUI_YELLOW);
    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_TOP);
    GUI_DispStringAt(ac, SIDE_X + SIDE_W / 2, y + 6);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "RUN", GUI_GREEN, GUI_BLACK);
  }
  else if (page == 4)
  {
    /* 分组决定写入哪一对文件：csv1 + bmp1 */
    y = SIDE_Y;
    ac[0] = 0;
    Str_App(ac, "GRP ");
    Str_Int(ac, storeGroup + 1u);
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, ac, C_BTN2, GUI_YELLOW);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "SAVE CSV", C_BTN, GUI_WHITE);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, "SAVE BMP", C_BTN, GUI_WHITE);

    /* 叠加哪一组保存的波形：REF - 表示关闭 */
    y += SIDE_STEP;
    if (refGroup < 0)
    {
      App_Button(SIDE_X, y, SIDE_W, SIDE_H, "REF -", C_BTN, GUI_WHITE);
    }
    else
    {
      ac[0] = 0;
      Str_App(ac, "REF ");
      Str_Int(ac, (unsigned)refGroup + 1u);
      App_Button(SIDE_X, y, SIDE_W, SIDE_H, ac, C_BTN2, GUI_YELLOW);
    }

    y += SIDE_STEP + 6;
    GUI_SetColor(GUI_YELLOW);
    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_TOP);
    GUI_DispStringAt(statusText[0] ? statusText : "", SIDE_X + SIDE_W / 2, y);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);
  }
  else
  {
    /* 第 5 页：光标。触摸拖动只能粗调，这里用按键一格一格地挪。 */
    static const char *apCurName[4] = { "X1", "X2", "Y1", "Y2" };

    y = SIDE_Y;
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, cursorOn ? "CUR on" : "CUR off",
               cursorOn ? C_BTN2 : C_BTN,
               cursorOn ? GUI_YELLOW : GUI_WHITE);

    /* 当前被按键控制的那条线 */
    y += SIDE_STEP;
    ac[0] = 0;
    Str_App(ac, "SEL ");
    Str_App(ac, apCurName[curSel & 3]);
    App_Button(SIDE_X, y, SIDE_W, SIDE_H, ac, C_BTN2, GUI_WHITE);

    /* 左右：移动竖光标；上下：移动横光标 */
    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W / 2, SIDE_H, "<", C_BTN, GUI_WHITE);
    App_Button(SIDE_X + SIDE_W / 2, y, SIDE_W / 2, SIDE_H, ">", C_BTN, GUI_WHITE);

    y += SIDE_STEP;
    App_Button(SIDE_X, y, SIDE_W / 2, SIDE_H, "^", C_BTN, GUI_WHITE);
    App_Button(SIDE_X + SIDE_W / 2, y, SIDE_W / 2, SIDE_H, "v", C_BTN, GUI_WHITE);

    y += SIDE_STEP + 10;
    GUI_SetColor(0x00808080);
    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_TOP);
    GUI_DispStringAt("step 1px", SIDE_X + SIDE_W / 2, y);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);
  }
}

/******************************************************************************
*  内容绘制（网格 + 波形 + 测量），绘制到 memory device 中
******************************************************************************/
#define XPOS(i)   (PLOT_X0 + (int)(((long)(i) * PLOT_W) / (N_DISP - 1)))

/* 垂直灵敏度：模拟区纵向共 YDIV 格（aVdiv 与其它文件作用域变量一起声明）。 */

/* 把毫伏转换为屏幕行。整个模拟区覆盖 YDIV 格、每格 aVdiv 毫伏，
   vCen 是中间行对应的电压，因此带偏置的波形也能保持居中，
   而不会跑出屏幕。 */
static int App_YA(int mv, int ch)
{
  int span = aVdiv[vIdx] * YDIV;             /* 满量程，单位 mV */
  int d    = vCen[ch] - mv;                  /* 正值 = 位于中心下方 */
  int y    = ANA_Y0 + ANA_H / 2 + (int)(((long)d * ANA_H) / span);

  if (y < ANA_Y0) y = ANA_Y0;                /* 超出范围：贴在边缘，
                                                类似削顶的示波器 */
  if (y > ANA_Y1) y = ANA_Y1;
  return y;
}

/* 可见窗口的中值，切换灵敏度时作为新的垂直中心，
   使带直流偏置的通道仍留在屏幕内 */
int App_WinMid(int ch)
{
  int i, idx, v, vMin, vMax;

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
  return (vMin + vMax) / 2;
}

/* 参考波形：把某一组保存时的那 400 个点叠加在当前波形下面。
   用同色系的暗色绘制，当前波形因此仍然清楚可辨。                            */
static void App_DrawRef(void)
{
  const SCOPE_SAMPLE *pRef;
  int i, x1, x2;

  if (refGroup < 0)
  {
    return;
  }
  pRef = Store_Ref((uint32_t)refGroup);
  if (pRef == 0)                 /* 该组还没保存过，没有可叠加的内容 */
  {
    return;
  }

  for (i = 0; i < (N_DISP - 1); i++)
  {
    x1 = XPOS(i);
    x2 = XPOS(i + 1);

    if (App_Vis(0))
    {
      GUI_SetColor(0x00006060);  /* 暗黄 = 参考 CH1 */
      GUI_DrawLine(x1, App_YA(pRef[i].ch1, 0), x2, App_YA(pRef[i + 1].ch1, 0));
    }
    if (App_Vis(1))
    {
      GUI_SetColor(0x00606000);  /* 暗青 = 参考 CH2 */
      GUI_DrawLine(x1, App_YA(pRef[i].ch2, 1), x2, App_YA(pRef[i + 1].ch2, 1));
    }
  }
}

/* 绘图区网格：垂直分隔线贯穿整个高度，水平线只画在模拟区，
   外加数字通道行的分隔线。 */
static void App_DrawGrid(void)
{
  int i, xp, row;

  GUI_SetColor(C_GRID);

  for (i = 0; i <= XDIV; i++)
  {
    xp = PLOT_X0 + i * DIV_X;
    GUI_DrawVLine(xp, ANA_Y0, DIG_Y1);
  }

  /* 模拟区的 5 条水平线 */
  for (i = 1; i <= YDIV; i++)
  {
    GUI_DrawHLine(ANA_Y0 + i * DIV_Y, PLOT_X0, PLOT_X1);
  }

  /* 数字区的分隔线与行线 */
  GUI_DrawHLine(DIG_Y0 - 1, PLOT_X0, PLOT_X1);
  for (i = 1; i < 3; i++)
  {
    row = DIG_Y0 + i * DIG_ROW_H;
    GUI_DrawHLine(row - 1, PLOT_X0, PLOT_X1);
  }
}

/* 把五条波形画成首尾相接的线段，使整个窗口成为一条连续折线。
   数字通道在两个相邻采样不同时补一条竖线，这样跳变沿才可见。 */
static void App_DrawTraces(void)
{
  int      i, x1, x2, y1, y2, yH[3], yL[3];
  int      idx, nextIdx;
  SCOPE_SAMPLE *pCur, *pNext;

  for (i = 0; i < 3; i++)
  {
    yH[i] = DIG_Y0 + i * DIG_ROW_H + 3;
    yL[i] = DIG_Y0 + i * DIG_ROW_H + 14;
  }

  idx = App_StartIndex();

  for (i = 0; i < (N_DISP - 1); i++)
  {
    nextIdx = idx + 1;
    if (nextIdx >= App_BufLen())
    {
      nextIdx = 0;
    }
    pCur  = &aSamples[idx];
    pNext = &aSamples[nextIdx];
    x1 = XPOS(i);
    x2 = XPOS(i + 1);

    if (App_Vis(0))
    {
      GUI_SetColor(C_CH1);
      GUI_DrawLine(x1, App_YA(pCur->ch1, 0), x2, App_YA(pNext->ch1, 0));
    }
    if (App_Vis(1))
    {
      GUI_SetColor(C_CH2);
      GUI_DrawLine(x1, App_YA(pCur->ch2, 1), x2, App_YA(pNext->ch2, 1));
    }

    if (App_Vis(2))
    {
      GUI_SetColor(C_D0);
      y1 = (pCur->dig & 1u) ? yH[0] : yL[0];
      y2 = (pNext->dig & 1u) ? yH[0] : yL[0];
      GUI_DrawLine(x1, y1, x2, y2);
      if (y1 != y2) GUI_DrawVLine(x2, yH[0], yL[0]);
    }
    if (App_Vis(3))
    {
      GUI_SetColor(C_D1);
      y1 = (pCur->dig & 2u) ? yH[1] : yL[1];
      y2 = (pNext->dig & 2u) ? yH[1] : yL[1];
      GUI_DrawLine(x1, y1, x2, y2);
      if (y1 != y2) GUI_DrawVLine(x2, yH[1], yL[1]);
    }
    if (App_Vis(4))
    {
      GUI_SetColor(C_D2);
      y1 = (pCur->dig & 4u) ? yH[2] : yL[2];
      y2 = (pNext->dig & 4u) ? yH[2] : yL[2];
      GUI_DrawLine(x1, y1, x2, y2);
      if (y1 != y2) GUI_DrawVLine(x2, yH[2], yL[2]);
    }

    idx = nextIdx;
  }

  /* 数字通道行左边缘的通道标记 */
  GUI_SetColor(C_D0); GUI_DispStringAt("D0", 1, DIG_Y0 + 3);
  GUI_SetColor(C_D1); GUI_DispStringAt("D1", 1, DIG_Y0 + DIG_ROW_H + 3);
  GUI_SetColor(C_D2); GUI_DispStringAt("D2", 1, DIG_Y0 + 2 * DIG_ROW_H + 3);

  /* 垂直档位，仅当与默认映射不同时显示 */
  if (vIdx != 0)
  {
    char ac[16];
    ac[0] = 0;
    Str_Int(ac, (unsigned)aVdiv[vIdx]);
    Str_App(ac, "mV/div");
    GUI_SetColor(0x00606060);
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_TOP);
    GUI_DispStringAt(ac, PLOT_X1 - 4, ANA_Y0 + 2);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);
  }

  /* 叠加的参考组号，与上面的档位分行显示（该组没保存过时不显示） */
  if ((refGroup >= 0) && (Store_Ref((uint32_t)refGroup) != 0))
  {
    char ac[12];
    ac[0] = 0;
    Str_App(ac, "REF ");
    Str_Int(ac, (unsigned)refGroup + 1u);
    GUI_SetColor(0x0000C0C0);
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_TOP);
    GUI_DispStringAt(ac, PLOT_X1 - 4, ANA_Y0 + 12);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);
  }
}

/******************************************************************************
*  测量光标：两条竖线（时间）与两条横线（电压），可触摸拖动
******************************************************************************/

/* 虚线按 4 像素一节手绘：emWin 的线型接口在不同配置下表现不一致，
   手绘能保证任何工程设置下外观相同。                                        */
static void App_DashV(int x, int y0, int y1)
{
  int y;

  for (y = y0; y <= y1; y += 4)
  {
    GUI_DrawVLine(x, y, ((y + 1) <= y1) ? (y + 1) : y1);
  }
}

static void App_DashH(int y, int x0, int x1)
{
  int x;

  for (x = x0; x <= x1; x += 4)
  {
    GUI_DrawHLine(y, x, ((x + 1) <= x1) ? (x + 1) : x1);
  }
}

/* 触发标记：模拟源画出触发电平线，数字源在对应行标一个 T。
   颜色偏暗，不会盖过波形，但能看出触发点到底定在哪里。                      */
static void App_DrawTrig(void)
{
  int y, row;

  if (!App_Vis(triggerSource))
  {
    return;                        /* 通道被隐藏时没有可对齐的波形 */
  }

  GUI_SetColor(0x000080C0);        /* 暗橙 */
  GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_TOP);

  if (triggerSource < 2)
  {
    y = App_YA(triggerLevelMv, triggerSource);
    App_DashH(y, PLOT_X0, PLOT_X1);
    GUI_DispStringAt("T", PLOT_X1 - 2, (y > (ANA_Y0 + 10)) ? (y - 10) : (y + 1));
  }
  else
  {
    row = triggerSource - 2;
    GUI_DispStringAt("T", PLOT_X1 - 2, DIG_Y0 + row * DIG_ROW_H + 3);
  }

  GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);
}

/* 绘制光标线，并在左上角给出两条竖线之间的时间差（及其倒数）
   和两条横线之间的电压差。                                                  */
static void App_DrawCursor(void)
{
  int  i, i0, i1, t, dt, dv;
  char ac[48];

  if (!cursorOn)
  {
    return;
  }

  /* 被按键选中的那条线画成亮白，其余保持灰色，便于看清正在调哪一条 */
  for (i = 0; i < 2; i++)
  {
    GUI_SetColor((curSel == i) ? GUI_WHITE : 0x00808080);
    App_DashV(XPOS(curX[i]), ANA_Y0, DIG_Y1);   /* 竖线贯穿整个绘图区 */
    GUI_SetColor((curSel == (2 + i)) ? GUI_WHITE : 0x00808080);
    App_DashH(App_YA(curY[i], 0), PLOT_X0, PLOT_X1);
  }

  /* 两条竖线之间的时间：点数 x 每点时间 */
  i0 = curX[0];
  i1 = curX[1];
  if (i1 < i0)
  {
    t  = i0;
    i0 = i1;
    i1 = t;
  }
  dt = (i1 - i0) * (int)(aTbUs[timebaseIdx] / SPP);

  dv = curY[1] - curY[0];
  if (dv < 0) dv = -dv;

  ac[0] = 0;
  Str_App(ac, "dt ");
  Str_Time(ac, dt);
  if (dt > 0)
  {
    Str_App(ac, "  1/dt ");
    Str_Freq(ac, 1000000 / dt);
  }
  Str_App(ac, "  dv ");
  Str_Volt2(ac, dv);
  Str_App(ac, "V");

  GUI_SetColor(GUI_WHITE);
  GUI_DispStringAt(ac, PLOT_X0 + 4, ANA_Y0 + 2);
}

/******************************************************************************
*  测量值行：固定列宽，互不重叠
******************************************************************************/

/* 底部测量条：每个通道一项，外加链路状态，全部用上面的小字符串辅助函数
   写入，避免使用 printf。 */
static void App_DrawInfo(void)
{
  int  vpp, fq, off, pw;
  char ac[48];

  /* 第 1 行：CH1 在 0，CH2 在 180，链路状态右对齐 */
  ac[0] = 0;
  Str_App(ac, "CH1 ");
  if (App_Vis(0))
  {
    App_MeasAnalog(0, &vpp, &fq, &off);
    Str_Volt2(ac, vpp);
    Str_App(ac, "V ");
    Str_Freq(ac, fq);
    Str_App(ac, " @");
    Str_Volt2(ac, off);
    Str_App(ac, "V");
  }
  else
  {
    Str_App(ac, "--");
  }
  GUI_SetColor(App_Vis(0) ? C_CH1 : C_OFF);
  GUI_DispStringAt(ac, 0, INFO1_Y);

  ac[0] = 0;
  Str_App(ac, "CH2 ");
  if (App_Vis(1))
  {
    App_MeasAnalog(1, &vpp, &fq, &off);
    Str_Volt2(ac, vpp);
    Str_App(ac, "V ");
    Str_Freq(ac, fq);
    Str_App(ac, " @");
    Str_Volt2(ac, off);
    Str_App(ac, "V");
  }
  else
  {
    Str_App(ac, "--");
  }
  GUI_SetColor(App_Vis(1) ? C_CH2 : C_OFF);
  GUI_DispStringAt(ac, 196, INFO1_Y);

  /* 必须在按钮列之前结束，否则会被覆盖 */
  GUI_SetColor(liveMode ? GUI_GREEN : GUI_RED);
  GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_TOP);
  GUI_DispStringAt(liveMode ? "LINK OK" : "NO LINK", PLOT_X1 - 2, INFO1_Y);
  GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_TOP);

  /* 第 2 行：三个数字通道分别在 0 / 164 / 328 */
  ac[0] = 0;
  Str_App(ac, "D0 ");
  if (App_Vis(2))
  {
    App_MeasDigital(0, &fq, &pw);
    Str_Freq(ac, fq);
    Str_App(ac, " pw");
    Str_Time(ac, pw);
  }
  else
  {
    Str_App(ac, "--");
  }
  GUI_SetColor(App_Vis(2) ? C_D0 : C_OFF);
  GUI_DispStringAt(ac, 0, INFO2_Y);

  ac[0] = 0;
  Str_App(ac, "D1 ");
  if (App_Vis(3))
  {
    App_MeasDigital(1, &fq, &pw);
    Str_Freq(ac, fq);
    Str_App(ac, " pw");
    Str_Time(ac, pw);
  }
  else
  {
    Str_App(ac, "--");
  }
  GUI_SetColor(App_Vis(3) ? C_D1 : C_OFF);
  GUI_DispStringAt(ac, 158, INFO2_Y);

  ac[0] = 0;
  Str_App(ac, "D2 ");
  if (App_Vis(4))
  {
    App_MeasDigital(2, &fq, &pw);
    Str_Freq(ac, fq);
    Str_App(ac, " pw");
    Str_Time(ac, pw);
  }
  else
  {
    Str_App(ac, "--");
  }
  GUI_SetColor(App_Vis(4) ? C_D2 : C_OFF);
  GUI_DispStringAt(ac, 316, INFO2_Y);
}

/******************************************************************************
*  链路活动条（底行）：采集链路的 RX / TX 以及 USB 下载链路。
*  计数变化时小方块点亮，因此一根断线还是波特率错误可以一眼区分。
******************************************************************************/

/* 单个活动指示灯：自上次刷新以来计数有变化时显示绿色。 */
static void App_Led(int x, int y, int on)
{
  GUI_SetColor(on ? GUI_GREEN : C_OFF);
  GUI_FillRect(x, y, x + 5, y + 5);
}

/* 底栏：两条链路的字节计数、指示灯以及最近一次发出的控制帧。
   最后绘制，因为它横跨整个宽度。 */
static void App_DrawStatus(void)
{
  static uint32_t lastRx, lastTx, lastPc;
  char ac[24];

  GUI_SetColor(C_BAR);
  GUI_FillRect(0, 260, SCR_W - 1, SCR_H - 1);

  GUI_SetColor(GUI_WHITE);

  App_Led(4, 262, gDaq.rxBytes != lastRx);
  GUI_DispStringAt("RX", 14, 261);
  ac[0] = 0;
  Str_Int(ac, (unsigned)gDaq.rxBytes);
  GUI_DispStringAt(ac, 32, 261);

  App_Led(124, 262, gDaq.txBytes != lastTx);
  GUI_DispStringAt("TX", 134, 261);
  ac[0] = 0;
  Str_Int(ac, (unsigned)gDaq.txBytes);
  GUI_DispStringAt(ac, 152, 261);

  App_Led(244, 262, gDaq.hostRxBytes != lastPc);
  GUI_DispStringAt("USB", 254, 261);
  ac[0] = 0;
  Str_Int(ac, (unsigned)gDaq.hostRxBytes);
  GUI_DispStringAt(ac, 278, 261);

  /* 最后发出的控制帧，例如 A3B1000C50D */
  GUI_DispStringAt("TX:", 332, 261);
  GUI_DispStringAt(gDaq.lastCtrl, 350, 261);

  lastRx = gDaq.rxBytes;
  lastTx = gDaq.txBytes;
  lastPc = gDaq.hostRxBytes;
}

/* 一次完整的屏幕刷新，离屏合成后一次性拷贝 */
void App_DrawContent(void)
{
  /* 先把所有内容合成到 memory device：LCD 只看到完成的画面，
     这正是消除直接绘制版本“波形绘制过程中可见”的闪烁的原因 */
  if (hMemDev)
  {
    GUI_MEMDEV_Select(hMemDev);
  }

  GUI_SetBkColor(GUI_BLACK);
  GUI_Clear();

  App_DrawTop();
  App_DrawGrid();
  App_DrawTrig();               /* 触发电平标记，画在波形之下 */
  App_DrawRef();                /* 参考波形画在当前波形之下 */
  App_DrawTraces();
  App_DrawCursor();             /* 光标压在波形之上 */
  App_DrawInfo();
  App_DrawSide();               /* 随页面变化的按钮列 */
  App_DrawStatus();             /* 最后绘制，因为它横跨整个宽度 */

  if (hMemDev)
  {
    GUI_MEMDEV_Select(0);
    GUI_MEMDEV_Write(hMemDev);
  }
}

/*************************** 文件结束 ****************************/
