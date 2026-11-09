/*
******************************************************************************
  文件        : DAQ_Util.c
  功能        : Remote DAQ 显示端的小型字符串辅助函数。
                printf 太慢且会引入庞大库，因此屏幕上显示的每个数字、写入
                导出 CSV 的每个数字都由这几个函数格式化。它们都在目标字符串
                的结尾零处追加，一行文本由对同一缓冲的若干次调用依次拼成。

  使用方      : DAQ_Draw.c（标签、测量值）与 DAQ_Store.c（CSV 文本）。
  平台        : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#include <stdint.h>
#include "RemoteDAQ_Private.h"

/******************************************************************************
*  小型字符串辅助函数（渲染循环中不使用 printf）
******************************************************************************/

/* 指向字符串的结尾零，便于调用者追加。 */
static char *Str_End(char *p)
{
  while (*p) p++;
  return p;
}

/* 原地连接两个字符串，目标缓冲必须有足够空间。 */
void Str_App(char *p, const char *s)
{
  char *q = Str_End(p);
  while (*s) *q++ = *s++;
  *q = 0;
}

/* 不使用 printf 追加无符号整数：printf 太慢，
   而且为了几位数字就要引入庞大库。                                          */
void Str_Int(char *p, unsigned v)
{
  char  t[8];
  int   n = 0;
  char *q;

  do
  {
    t[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v);

  q = Str_End(p);
  while (n) *q++ = t[--n];
  *q = 0;
}

/* 两位小数：1.10 */
void Str_Volt2(char *p, int mv)
{
  char *q;
  int   ip, fp;

  if (mv < 0) mv = 0;
  ip = mv / 1000;
  fp = (mv % 1000) / 10;
  Str_Int(p, (unsigned)ip);
  q = Str_End(p);
  *q++ = '.';
  *q++ = (char)('0' + (fp / 10));
  *q++ = (char)('0' + (fp % 10));
  *q = 0;
}

/* 频率：375Hz / 1.30kHz */
void Str_Freq(char *p, int hz)
{
  char *q;

  if (hz < 0) hz = 0;
  if (hz >= 1000)
  {
    Str_Int(p, (unsigned)(hz / 1000));
    q = Str_End(p);
    *q++ = '.';
    *q++ = (char)('0' + ((hz % 1000) / 100));
    *q++ = (char)('0' + (((hz % 1000) / 10) % 10));
    *q = 0;
    Str_App(p, "kHz");
  }
  else
  {
    Str_Int(p, (unsigned)hz);
    Str_App(p, "Hz");
  }
}

/* 脉宽：498us / 1.00ms */
void Str_Time(char *p, int us)
{
  char *q;

  if (us >= 1000)
  {
    Str_Int(p, (unsigned)(us / 1000));
    q = Str_End(p);
    *q++ = '.';
    *q++ = (char)('0' + ((us % 1000) / 100));
    *q++ = (char)('0' + (((us % 1000) / 10) % 10));
    *q = 0;
    Str_App(p, "ms");
  }
  else
  {
    Str_Int(p, (unsigned)us);
    Str_App(p, "us");
  }
}

/* 1.0k / 50 */
void Str_Short(char *p, int v)
{
  if (v >= 1000)
  {
    Str_Int(p, (unsigned)(v / 1000));
    Str_App(p, ".");
    {
      char ac[2];
      ac[0] = (char)('0' + ((v % 1000) / 100));
      ac[1] = 0;
      Str_App(p, ac);
    }
    Str_App(p, "k");
  }
  else
  {
    Str_Int(p, (unsigned)v);
  }
}

/*************************** 文件结束 ****************************/
