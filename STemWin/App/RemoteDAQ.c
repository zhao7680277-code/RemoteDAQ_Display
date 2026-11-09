/*
******************************************************************************
  文件          : RemoteDAQ.c
  用途          : Remote DAQ 显示端 - 示波器风格波形界面
                2 路模拟 + 3 路数字波形，实时数据经 USART6 传输

  布局          : 绘图区占据屏幕左侧，右侧列放上下文按钮，顶栏放全局按钮
                （时间基准、保存、页码）。翻页只替换右侧按钮，波形和测量行
                始终留在屏幕上，用户不会失去对信号的观察。

  数据流        : USART6 -> DMA 循环缓冲 -> 帧同步 + CRC -> gDaq
                -> 本文件的采样环形缓冲 -> 屏幕窗口
                -> memory device 合成 -> 一次完整的 LCD 拷贝。
                没有有效帧时由内置仿真器填充环形缓冲，屏幕不会空白。

  平台          : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#include <stdint.h>
#include "GUI.h"
#include "DAQ_Link.h"
#include "RemoteDAQ_Private.h"

/******************************************************************************
*  私有变量
*
*  它们是整个应用的状态，因此在这里定义一次，并在 RemoteDAQ_Private.h 中
*  声明为 extern。采样环形缓冲本身由 DAQ_Sim.c 填充、DAQ_Draw.c 绘制、
*  DAQ_Measure.c 测量、DAQ_Store.c 导出，所以缓冲必须放在这些文件之外。
******************************************************************************/

/* 各通道的波形颜色，以及在波形上仍清晰可读的文字颜色 */
const int aChColor[5] = { GUI_YELLOW, GUI_CYAN, GUI_GREEN, GUI_BLUE, GUI_MAGENTA };
const int aChFg[5]    = { GUI_BLACK,  GUI_BLACK, GUI_BLACK, GUI_WHITE, GUI_BLACK };

SCOPE_SAMPLE aSamples[N_SAMP];
uint16_t     sampleWriteIdx;   /* 写索引（仿真模式） */
uint32_t     simTimeUs;        /* 下一样本的生成时刻 */
uint32_t     simPhaseA, simPhaseB;   /* 相位累加器 */
uint32_t     simStepA, simStepB;     /* 相位步进 */

int liveMode     = 0;           /* 1 = 正在绘制 USART6 收到的帧 */
int timebaseIdx  = 2;           /* 默认 1 ms/格 */
int runPaused    = 0;
int triggerMode  = 0;           /* 0 自动，1 正常，2 单次 */
int triggerSource = 0;          /* 0..4 = CH1 CH2 D0 D1 D2 */
int triggerLevelMv = 1500;      /* mV */
int page         = 1;           /* 1..4 */
int channelOn[5] = { 1, 1, 1, 1, 1 };   /* CH1, CH2, D0, D1, D2 */
int pwmFreq      = 1000;
int pwmDuty      = 50;
char statusText[16] = "";
uint32_t storeGroup;            /* 选中的文件组，0..3（显示为 1..4） */

int vIdx    = 0;                /* 垂直灵敏度，aVdiv 的下标 */
int vCen[2] = { 1650, 1650 };   /* 模拟区中心的电压，单位 mV    */

/* 附加功能状态：参考波形叠加与测量光标 */
int refGroup = -1;                              /* -1 = 关闭，0..3 = 叠加该组 */
int cursorOn = 0;
int curX[2]  = { N_DISP / 3, (2 * N_DISP) / 3 }; /* 竖光标，屏上采样索引 */
int curY[2]  = { 1250, 2050 };                   /* 横光标，mV */
int curSel   = 0;               /* 按键移动的光标线：0/1 = X1/X2，2/3 = Y1/Y2 */

int trigFound = 0;              /* 最近一次窗口搜索是否找到触发点 */

/* 垂直灵敏度档位，模拟区纵向共 YDIV 格。660 mV/格 = 3300 mV / 5 格，
   即原先固定的 0..3300 mV 映射；更小的档位用于麦克风模块这类小信号，
   否则其原始摆幅只能覆盖几个像素。 */
const int aVdiv[5] = { 660, 330, 165, 66, 33 };


GUI_MEMDEV_Handle hMemDev;  /* 0 = 不使用 memory device，直接绘制 */

const uint32_t aTbUs[5]    = { 10000, 5000, 1000, 500, 100 };
const char    *apTbName[5] = { "TB:10ms", "TB:5ms", "TB:1ms", "TB:500us", "TB:100us" };
const char    *apTrig[3]   = { "Auto", "Norm", "Sing" };

/******************************************************************************
*  动作
******************************************************************************/

/* 把采集板设置为与显示端相同的时间基准和 PWM 参数，
   保证两端始终一致地显示同一信号。 */
static void App_SendControl(void)
{
  /* 控制帧 v2.0：A<mode>B<freq>C<duty>D
     mode  = 采集档位 1..5（与时间基准按钮编号一致）
     freq  = 100 .. 10000 Hz
     duty  = 0 .. 100 %
     只发送这一种帧：链路上的其它字符会被采集板解析器当作未知字符。 */
  DAQ_Link_SendControl((uint8_t)(timebaseIdx + 1), (uint16_t)pwmFreq, (uint8_t)pwmDuty);
}

/* 应用新的时间基准：限制索引范围，仿真数据时重建仿真器，
   并把变化通知采集板。 */
static void App_SetTimebase(int idx)
{
  if (idx < 0) idx = 0;
  if (idx > 4) idx = 4;
  if (idx == timebaseIdx)
  {
    return;
  }

  timebaseIdx = idx;

  /* 仿真器必须重建，否则波形永远不会变化 */
  if (!liveMode)
  {
    App_ResetBuffer();
  }

  /* 采集板必须切换到相同的时间基准（1..5） */
  App_SendControl();

  App_DrawContent();
}

/* 用按键微调光标：每次移动约 1 个像素。
   触摸拖动适合粗调，绘制区里 1 像素很难点准，所以细调交给按键。 */
static void App_NudgeCursor(int id)
{
  int i, step;

  if (!cursorOn)                  /* 还没打开光标？按键直接打开 */
  {
    cursorOn = 1;
    curSel   = 0;
  }

  if (curSel < 2)                 /* 竖光标：左右移动，1 点 = 1 像素 */
  {
    if (id == ID_CL) curX[curSel]--;
    if (id == ID_CR) curX[curSel]++;

    if (curX[curSel] < 0)              curX[curSel] = 0;
    if (curX[curSel] > (N_DISP - 1))   curX[curSel] = N_DISP - 1;
  }
  else                            /* 横光标：上下移动，步长取 1 像素对应的电压 */
  {
    i    = curSel - 2;
    step = (aVdiv[vIdx] * YDIV) / ANA_H;    /* mV / 像素 */
    if (step < 1) step = 1;

    if (id == ID_CU) curY[i] += step;       /* 屏幕上移 = 电压升高 */
    if (id == ID_CD) curY[i] -= step;

    if (curY[i] < 0)    curY[i] = 0;
    if (curY[i] > 3300) curY[i] = 3300;
  }

  App_DrawContent();
}

/* 执行一个按钮的动作。标识符与下面的触摸处理共用，
   这样几何布局与行为始终保持一致。 */
static void App_Action(int id)
{
  switch (id)
  {
    case ID_PAGE:
      page = (page % 5) + 1;
      App_DrawContent();          /* 波形保持可见 */
      break;

    case ID_TB:
      App_SetTimebase((timebaseIdx + 1) % 5);
      break;

    case ID_CH1:
    case ID_CH2:
    case ID_D0:
    case ID_D1:
    case ID_D2:
      channelOn[id - ID_CH1] = !channelOn[id - ID_CH1];
      App_DrawContent();
      break;

    case ID_TM:
      App_SetTimebase(timebaseIdx - 1);
      break;

    case ID_TP:
      App_SetTimebase(timebaseIdx + 1);
      break;

    case ID_RUN:
      runPaused = !runPaused;
      break;

    case ID_VD:
      /* 切到下一档灵敏度，并按各通道当前显示的信号重新居中，
         避免直流偏置把波形推出屏幕 */
      vIdx = (vIdx + 1) % 5;
      vCen[0] = App_WinMid(0);
      vCen[1] = App_WinMid(1);
      App_DrawContent();
      break;

    case ID_FM:
      pwmFreq -= (pwmFreq <= 1000) ? 100 : 1000;
      if (pwmFreq < 100) pwmFreq = 100;
      App_SendControl();
      App_DrawContent();
      break;

    case ID_FP:
      pwmFreq += (pwmFreq < 1000) ? 100 : 1000;
      if (pwmFreq > 10000) pwmFreq = 10000;
      App_SendControl();
      App_DrawContent();
      break;

    case ID_DM:
      pwmDuty -= 5;
      if (pwmDuty < 0) pwmDuty = 0;      /* 0 % = 无输出，v2.0 允许 */
      App_SendControl();
      App_DrawContent();
      break;

    case ID_DP:
      pwmDuty += 5;
      if (pwmDuty > 100) pwmDuty = 100;  /* 100 % = 直流电平 */
      App_SendControl();
      App_DrawContent();
      break;

    case ID_TRIG:
      triggerMode = (triggerMode + 1) % 3;
      App_SendControl();
      App_DrawContent();
      break;

    case ID_TSRC:
      triggerSource = (triggerSource + 1) % 5;
      App_SendControl();
      App_DrawContent();
      break;

    case ID_TLM:
      triggerLevelMv -= 100;
      if (triggerLevelMv < 0) triggerLevelMv = 0;
      App_SendControl();
      App_DrawContent();
      break;

    case ID_TLP:
      triggerLevelMv += 100;
      if (triggerLevelMv > 3300) triggerLevelMv = 3300;
      App_SendControl();
      App_DrawContent();
      break;

    case ID_GRP:
      storeGroup = (storeGroup + 1u) % STORE_GROUPS;
      statusText[0] = 0;          /* 之前的结果属于另一个分组 */
      App_DrawContent();
      break;

    case ID_CSV:
      Store_Save(ST_CSV);         /* 当前分组，显示 "CSVn OK" */
      break;

    case ID_BMP:
      Store_Save(ST_BMP);         /* 当前分组，显示 "BMPn OK" */
      break;

    case ID_REF:
      /* 关闭 -> 第 1 组 -> ... -> 第 4 组 -> 关闭。
         参考波形在保存该组时留下，未保存过的组不显示任何内容。 */
      refGroup++;
      if (refGroup >= (int)STORE_GROUPS)
      {
        refGroup = -1;
      }
      App_DrawContent();
      break;

    case ID_CUR:
      cursorOn = !cursorOn;
      if (cursorOn)
      {
        /* 竖线放在窗口 1/3 与 2/3，横线围绕 CH1 中心上下分开 */
        curX[0] = N_DISP / 3;
        curX[1] = (2 * N_DISP) / 3;
        curY[0] = vCen[0] - 400;
        curY[1] = vCen[0] + 400;
        if (curY[0] < 0)    curY[0] = 0;
        if (curY[1] > 3300) curY[1] = 3300;
      }
      App_DrawContent();
      break;

    case ID_SEL:
      /* 依次选中 X1、X2、Y1、Y2，按键微调的就是这一条 */
      curSel = (curSel + 1) & 3;
      if (!cursorOn)
      {
        cursorOn = 1;
      }
      App_DrawContent();
      break;

    case ID_CL:
    case ID_CR:
    case ID_CU:
    case ID_CD:
      App_NudgeCursor(id);
      break;

    default:
      break;
  }
}

/******************************************************************************
*  触摸处理
******************************************************************************/

/* 单个矩形的命中测试。右边缘和下边缘不包含在内，
   避免相邻按钮在公共边界上同时响应。 */
static int App_Hit(const GUI_PID_STATE *pState, int x, int y, int w, int h)
{
  return (pState->x >= x) && (pState->x < (x + w)) &&
         (pState->y >= y) && (pState->y < (y + h));
}

/******************************************************************************
*  测量光标：屏幕坐标与光标数值的互换算
*  换算公式与 DAQ_Draw.c 中 App_YA / XPOS 完全一致，光标才能准确压在波形上。
******************************************************************************/

/* 电压 -> 屏幕行（以 CH1 的垂直中心为准，与波形绘制相同） */
static int App_CurYA(int mv)
{
  int span = aVdiv[vIdx] * YDIV;
  int y    = ANA_Y0 + ANA_H / 2 +
             (int)(((long)(vCen[0] - mv) * ANA_H) / span);

  if (y < ANA_Y0) y = ANA_Y0;
  if (y > ANA_Y1) y = ANA_Y1;
  return y;
}

/* 屏幕行 -> 电压，超出模拟区时钳在 0..3300 mV */
static int App_CurMV(int y)
{
  int span = aVdiv[vIdx] * YDIV;
  int mv   = vCen[0] -
             (int)(((long)(y - (ANA_Y0 + ANA_H / 2)) * span) / ANA_H);

  if (mv < 0)    mv = 0;
  if (mv > 3300) mv = 3300;
  return mv;
}

/* 采样索引 -> 屏幕列 */
static int App_CurXPos(int i)
{
  return PLOT_X0 + (int)(((long)i * PLOT_W) / (N_DISP - 1));
}

/* 屏幕列 -> 采样索引，超出绘图区时钳在窗口两端 */
static int App_CurIdx(int x)
{
  int i = (int)(((long)(x - PLOT_X0) * (N_DISP - 1)) / PLOT_W);

  if (i < 0) i = 0;
  if (i > (N_DISP - 1)) i = N_DISP - 1;
  return i;
}

#define CUR_PICK  24        /* 触点离光标线多远仍算选中，单位：像素 */

/* 按下时挑出离触点最近的光标线：0/1 = 竖线，2/3 = 横线。
   都不在附近时返回 -1，触摸随后按普通按钮处理。 */
static int App_PickCursor(const GUI_PID_STATE *pState)
{
  int d, best = -1, bestD = CUR_PICK;

  d = pState->x - App_CurXPos(curX[0]);
  if (d < 0) d = -d;
  if (d < bestD) { bestD = d; best = 0; }

  d = pState->x - App_CurXPos(curX[1]);
  if (d < 0) d = -d;
  if (d <= bestD) { bestD = d; best = 1; }

  if (pState->y < ANA_Y1)          /* 横线只存在于模拟区 */
  {
    d = pState->y - App_CurYA(curY[0]);
    if (d < 0) d = -d;
    if (d < bestD) { bestD = d; best = 2; }

    d = pState->y - App_CurYA(curY[1]);
    if (d < 0) d = -d;
    if (d <= bestD) { best = 3; }
  }
  return best;
}

/* 把选中的光标线移到触点位置 */
static void App_MoveCursor(int sel, const GUI_PID_STATE *pState)
{
  if (sel < 2)
  {
    curX[sel] = App_CurIdx(pState->x);
  }
  else
  {
    curY[sel - 2] = App_CurMV(pState->y);
  }
}

/* 读取触摸面板，把一次新的按下转换为按钮动作。只使用按下沿，
   因此按住手指不会重复触发。 */
static void App_HandleTouch(void)
{
  static int wasPressed = 0;
  static int dragCur    = -1;      /* 正在拖动的光标线，-1 = 没有 */
  GUI_PID_STATE pState;
  int y;

  GUI_PID_GetState(&pState);

  /* 光标拖动优先于按钮：在绘图区内按下时选中最近的线，
     按住期间一直跟随手指。 */
  if (cursorOn && pState.Pressed)
  {
    if (dragCur < 0)
    {
      if ((pState.x >= PLOT_X0) && (pState.x < PLOT_X1) &&
          (pState.y >= ANA_Y0) && (pState.y < DIG_Y1))
      {
        dragCur = App_PickCursor(&pState);
      }
    }
    if (dragCur >= 0)
    {
      App_MoveCursor(dragCur, &pState);
      App_DrawContent();
      wasPressed = 1;
      return;
    }
  }
  else
  {
    dragCur = -1;
  }

  if (pState.Pressed && !wasPressed)
  {
    if (App_Hit(&pState, 428, 3, 48, 16))
    {
      App_Action(ID_PAGE);
    }
    else if (App_Hit(&pState,  70, 3, 62, 16))
    {
      App_Action(ID_CSV);
    }
    else if (App_Hit(&pState, 134, 3, 62, 16))
    {
      App_Action(ID_BMP);
    }
    else if (App_Hit(&pState, 202, 3, 64, 16))
    {
      App_Action(ID_TB);
    }
    else if (page == 1)
    {
      /* CH1 CH2 D0 D1 D2 T- T+ RUN */
      for (y = 0; y < 8; y++)
      {
        if (App_Hit(&pState, SIDE_X, SIDE_Y + y * SIDE_STEP, SIDE_W, SIDE_H))
        {
          App_Action((y < 5) ? (ID_CH1 + y)
                             : ((y == 5) ? ID_TM
                                         : ((y == 6) ? ID_TP : ID_RUN)));
          break;
        }
      }
    }
    else if (page == 2)
    {
      if (App_Hit(&pState, SIDE_X, SIDE_Y + 1 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_FM);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 2 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_FP);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 4 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_DM);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 5 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_DP);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 6 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_RUN);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 7 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_VD);
      }
    }
    else if (page == 3)
    {
      if (App_Hit(&pState, SIDE_X, SIDE_Y, SIDE_W, SIDE_H))
      {
        App_Action(ID_TRIG);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 1 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_TSRC);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 2 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_TLM);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 3 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_TLP);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 5 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_RUN);
      }
    }
    else if (page == 4)
    {
      if (App_Hit(&pState, SIDE_X, SIDE_Y, SIDE_W, SIDE_H))
      {
        App_Action(ID_GRP);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 1 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_CSV);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 2 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_BMP);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 3 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_REF);
      }
    }
    else
    {
      /* 第 5 页：光标开关、选择哪条线、四方向微调（左右两个按钮各占半格） */
      if (App_Hit(&pState, SIDE_X, SIDE_Y, SIDE_W, SIDE_H))
      {
        App_Action(ID_CUR);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 1 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(ID_SEL);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 2 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(App_Hit(&pState, SIDE_X + SIDE_W / 2, SIDE_Y + 2 * SIDE_STEP,
                           SIDE_W / 2, SIDE_H) ? ID_CR : ID_CL);
      }
      else if (App_Hit(&pState, SIDE_X, SIDE_Y + 3 * SIDE_STEP, SIDE_W, SIDE_H))
      {
        App_Action(App_Hit(&pState, SIDE_X + SIDE_W / 2, SIDE_Y + 3 * SIDE_STEP,
                           SIDE_W / 2, SIDE_H) ? ID_CD : ID_CU);
      }
    }
  }
  wasPressed = pState.Pressed;
}

/******************************************************************************
*  MainTask（GUI 初始化后由 main.c 调用一次）
******************************************************************************/

/* 应用入口：先初始化两条链路和存储，然后永久运行轮询 / 测量 / 绘制循环。
   所有工作都在主上下文完成，不从中断里绘制。 */
void MainTask(void)
{
  uint32_t lastDraw;
  uint32_t lastFrameNo;
  int      linkUp;

  GUI_SetFont(&GUI_Font6x8_ASCII);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetBkColor(GUI_BLACK);
  GUI_SetColor(GUI_WHITE);
  GUI_Clear();

  DAQ_Link_Init();          /* 测量链路           : USART6 */
  Host_Init();              /* 文件下载链路       : USART1 */
  Store_Init();             /* SDRAM 中的 4 个文件槽            */

  /* 离屏缓冲，消除示波器画面的闪烁 */
  hMemDev = GUI_MEMDEV_Create(CONTENT_X, CONTENT_Y, CONTENT_W, CONTENT_H);

  App_ResetBuffer();
  page = 1;

  App_DrawContent();

  /* 上电时发送一帧控制数据：证明 USART6 的 TX 确实从引脚发出
     （TX 计数不再是 0），同时把采集板设到相同的时间基准，
     以防它在上电后等待命令才发送数据 */
  App_SendControl();

  lastDraw    = HAL_GetTick();
  lastFrameNo = gDaq.frames;

  for (;;)
  {
    DAQ_Link_Poll();
    Store_Poll();             /* 来自 PC 的下载命令 */
    App_HandleTouch();

    /* 只要解析到一帧有效数据，且最后一帧的时间不超过 LINK_TIMEOUT，
       就认为链路正常。gDaq.frames / gDaq.lastTick 由解析器写入，
       所以这里完全不依赖 newFrame 标志。 */
    linkUp = (gDaq.frames != 0u) &&
             ((HAL_GetTick() - gDaq.lastTick) <= LINK_TIMEOUT);

    /* 链路丢失 -> 回落到内置仿真 */
    if (liveMode && !linkUp)
    {
      liveMode = 0;
      App_ResetBuffer();
      App_DrawContent();
    }

    /* 链路恢复但仍在绘制仿真数据 -> 切换为实时数据。
       App_LoadFrame() 是唯一设置 liveMode 的地方，而它以前只在
       “已经是实时”的分支里被调用，所以画面一直停留在仿真波形上，
       始终显示 NO LINK。 */
    if (!liveMode && linkUp)
    {
      App_LoadFrame();
      lastFrameNo = gDaq.frames;
      lastDraw    = HAL_GetTick();
      App_DrawContent();
    }

    /* 两种模式都按固定速率重绘。实时模式下数据到达可能快于屏幕合成速度，
       跟随到达速率正是刷新看起来不均匀的原因：始终使用最新帧，
       但每 DRAW_PERIOD 才用一次。 */
    if (liveMode)
    {
      /* 有新帧：正常 30 ms 节奏。没有新帧：放慢刷新，
         让 RX / TX 计数及指示灯在没有数据时仍有变化
         （否则画面会完全冻结）。 */
      if (!runPaused && ((HAL_GetTick() - lastDraw) >=
                         ((gDaq.frames != lastFrameNo) ? DRAW_PERIOD : STATUS_PERIOD)))
      {
        int drawThisFrame = 1;

        if (gDaq.frames != lastFrameNo)
        {
          lastFrameNo = gDaq.frames;
          App_LoadFrame();

          /* 触发方式在这里真正生效：
             Auto   - 找不到触发点也照常显示（App_StartIndex 会退回中点）
             Normal - 本帧没有触发就保持上一帧，波形停在最后一次触发上
             Single - 抓到一次触发后立即停住，按 RUN 重新武装 */
          App_StartIndex();                    /* 顺带更新 trigFound */

          if ((triggerMode == 1) && !trigFound)
          {
            drawThisFrame = 0;
          }
          else if ((triggerMode == 2) && trigFound)
          {
            runPaused = 1;
          }
        }

        lastDraw = HAL_GetTick();
        if (drawThisFrame)
        {
          App_DrawContent();
        }
      }
    }
    else if (!runPaused)
    {
      if ((HAL_GetTick() - lastDraw) >= DRAW_PERIOD)
      {
        uint32_t elapsed    = HAL_GetTick() - lastDraw;
        int      newSamples = (int)(elapsed * 1000u / (uint32_t)(aTbUs[timebaseIdx] / SPP));

        if (newSamples < 1) newSamples = 1;
        if (newSamples > N_SAMP) newSamples = N_SAMP;

        lastDraw = HAL_GetTick();
        App_GenSamples(newSamples);

        /* 仿真模式下触发方式同样生效，否则演示时看不出区别 */
        App_StartIndex();
        if ((triggerMode != 1) || trigFound)
        {
          App_DrawContent();
        }
        if ((triggerMode == 2) && trigFound)
        {
          runPaused = 1;             /* 单次：抓到一屏就停住 */
        }
      }
    }

    GUI_Delay(5);
  }
}

/*************************** 文件结束 ****************************/
