/*
******************************************************************************
  文件          : RemoteDAQ_Private.h
  用途          : Remote DAQ 显示应用的内部头文件。
                它集中了六个模块共享的内容：屏幕布局常量、按钮标识、
                私有类型、状态变量（在 RemoteDAQ.c 中定义一次），以及
                跨模块调用的函数。

  说明          : 本头文件只被应用的 RemoteDAQ*.c / DAQ_*.c 模块包含，
                工程其它部分不会包含它。
  平台          : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#ifndef __REMOTE_DAQ_PRIVATE_H
#define __REMOTE_DAQ_PRIVATE_H

#include <stdint.h>
#include "GUI.h"
#include "DAQ_Link.h"

/******************************************************************************
*  布局 (480 x 272)
******************************************************************************/
#define SCR_W         480
#define SCR_H         272

#define TOP_H         22

/* 绘图区：占满左侧宽度，从 x = 0 开始，到按钮列起始处结束 */
#define PLOT_X0       0
#define PLOT_X1       420
#define PLOT_W        (PLOT_X1 - PLOT_X0)      /* 420 */
#define XDIV          8
#define DIV_X         (PLOT_W / XDIV)          /* 52 */

/* 模拟通道区：5 条水平网格线 */
#define ANA_Y0        22
#define ANA_Y1        157
#define ANA_H         (ANA_Y1 - ANA_Y0)        /* 135 */
#define YDIV          5
#define DIV_Y         (ANA_H / YDIV)           /* 27 */

/* 数字通道区：3 行 */
#define DIG_Y0        162
#define DIG_ROW_H     21
#define DIG_Y1        (DIG_Y0 + 3 * DIG_ROW_H) /* 225 */

/* 右侧按钮列 */
#define SIDE_X        424
#define SIDE_W        52
#define SIDE_H        20
#define SIDE_Y        28
#define SIDE_STEP     25

/* 测量值行 */
#define INFO1_Y       234
#define INFO2_Y       250

#define SPP           50                       /* 每格采样点数 */
#define N_DISP        (XDIV * SPP)             /* 屏上显示 400 点 */

/* 触发点固定放在窗口左侧 1/5 处，这样触发沿前后的波形都能看到，
   而不是把触发沿挤在屏幕最左边 */
#define TRIG_POS      (N_DISP / 5)

/* 采集环形缓冲必须明显大于屏幕窗口：触发搜索要提前停下，
   保证整个显示窗口都位于写指针之前。否则波形会越过最旧的样本，
   时间轴回跳，屏幕上出现接缝。 */
#define N_SAMP        1600

/* 整屏先在 memory device 中合成，再一次性拷贝到屏幕，
   这正是消除直接绘制版本闪烁的原因 */
#define CONTENT_X     0
#define CONTENT_Y     0
#define CONTENT_W     SCR_W
#define CONTENT_H     SCR_H

#define LINK_TIMEOUT  1500                     /* 超过该时间无帧即判为断链 */
#define DRAW_PERIOD   30                       /* 两次屏幕更新的间隔 */
#define STATUS_PERIOD 100                      /* 空闲刷新，让指示灯保持活动 */

/******************************************************************************
*  颜色
******************************************************************************/
#define C_BAR         0x00202020
#define C_FRAME       0x00606060
#define C_GRID        0x00282828
#define C_BTN         0x00404040
#define C_BTN2        0x00505050
#define C_OFF         0x00606060

#define C_CH1         aChColor[0]
#define C_CH2         aChColor[1]
#define C_D0          aChColor[2]
#define C_D1          aChColor[3]
#define C_D2          aChColor[4]

/******************************************************************************
*  按钮标识
******************************************************************************/
#define ID_PAGE       1
#define ID_CH1        2
#define ID_CH2        3
#define ID_D0         4
#define ID_D1         5
#define ID_D2         6
#define ID_TM         7
#define ID_TP         8
#define ID_RUN        9
#define ID_FP         10
#define ID_FM         11
#define ID_DP         12
#define ID_DM         13
#define ID_TRIG       14
#define ID_CSV        15
#define ID_BMP        16
#define ID_TB         17
#define ID_TSRC       18
#define ID_TLM        19
#define ID_TLP        20
#define ID_GRP        21
#define ID_VD         22
#define ID_REF        23
#define ID_CUR        24
#define ID_SEL        25
#define ID_CL         26
#define ID_CR         27
#define ID_CU         28
#define ID_CD         29

/******************************************************************************
*  存储：两种文件类型和分组数量，RemoteDAQ.c 中的按钮动作也会用到
******************************************************************************/
#define STORE_GROUPS    4u
#define ST_CSV          1u
#define ST_BMP          2u

/******************************************************************************
*  私有类型
******************************************************************************/
typedef struct {
  uint16_t ch1;      /* CH1，单位 mV，0..3300 */
  uint16_t ch2;      /* CH2，单位 mV，0..3300 */
  uint8_t  dig;      /* bit0 = D0，bit1 = D1，bit2 = D2 */
} SCOPE_SAMPLE;

typedef struct {
  int x, y, w, h, id;
} BUTTON;

/******************************************************************************
*  私有变量（在 RemoteDAQ.c 中只定义一次）
******************************************************************************/
extern SCOPE_SAMPLE aSamples[N_SAMP];
extern uint16_t     sampleWriteIdx;   /* 写索引（仿真模式） */
extern uint32_t     simTimeUs;        /* 下一样本的生成时刻 */
extern uint32_t     simPhaseA, simPhaseB;   /* 相位累加器 */
extern uint32_t     simStepA, simStepB;     /* 相位步进 */

extern int liveMode;                   /* 1 = 正在绘制 USART6 收到的帧 */
extern int timebaseIdx;                /* 默认 1 ms/格 */
extern int runPaused;
extern int triggerMode;                /* 0 自动，1 正常，2 单次 */
extern int triggerSource;              /* 0..4 = CH1 CH2 D0 D1 D2 */
extern int triggerLevelMv;             /* mV */
extern int page;                       /* 1..4 */
extern int channelOn[5];               /* CH1, CH2, D0, D1, D2 */
extern int pwmFreq;
extern int pwmDuty;
extern char statusText[16];
extern uint32_t storeGroup;            /* 选中的文件组，0..3（显示为 1..4） */

extern int vIdx;                       /* 垂直灵敏度，aVdiv 的下标 */
extern int vCen[2];                    /* 模拟区中心的电压，单位 mV */

extern int refGroup;                   /* -1 = 关闭；0..3 = 叠加该组存下的波形 */
extern int cursorOn;                   /* 1 = 显示测量光标 */
extern int curX[2];                    /* 两条竖光标，单位：屏上采样索引 */
extern int curY[2];                    /* 两条横光标，单位：mV */
extern int curSel;                     /* 按键移动的那条：0/1 = X1/X2，2/3 = Y1/Y2 */
extern int trigFound;                  /* 最近一次窗口搜索是否真的找到了触发点 */

/* 各通道的波形颜色，以及在波形上仍清晰可读的文字颜色 */
extern const int aChColor[5];
extern const int aChFg[5];

/* 垂直灵敏度档位，模拟区纵向共 YDIV 格 */
extern const int aVdiv[5];

extern GUI_MEMDEV_Handle hMemDev;      /* 0 = 不使用 memory device，直接绘制 */

extern const uint32_t aTbUs[5];
extern const char    *apTbName[5];
extern const char    *apTrig[3];

/******************************************************************************
*  模块间共享的函数
******************************************************************************/

/* DAQ_Util.c - 小型字符串辅助函数，渲染循环中不使用 printf */
void  Str_App(char *p, const char *s);
void  Str_Int(char *p, unsigned v);
void  Str_Volt2(char *p, int mv);
void  Str_Freq(char *p, int hz);
void  Str_Time(char *p, int us);
void  Str_Short(char *p, int v);

/* DAQ_Sim.c - 采样环形缓冲与内置仿真器 */
void App_GenSamples(int n);
void App_ResetBuffer(void);
void App_LoadFrame(void);
int  App_Vis(int i);
int  App_BufLen(void);
int  App_StartIndex(void);

/* DAQ_Measure.c - 对可见窗口做测量 */
void App_MeasAnalog(int ch, int *pVpp, int *pFreq, int *pOff);
void App_MeasDigital(int ch, int *pFreq, int *pPw);

/* DAQ_Draw.c - 整屏合成 */
void App_DrawContent(void);
int  App_WinMid(int ch);

/* DAQ_Store.c - 板载存储与下载协议 */
void Store_Init(void);
void Store_Save(uint32_t type);
void Store_Poll(void);
const SCOPE_SAMPLE *Store_Ref(uint32_t g);

#endif /* __REMOTE_DAQ_PRIVATE_H */
