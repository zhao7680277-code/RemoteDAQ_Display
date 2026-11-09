/*
******************************************************************************
  文件        : DAQ_Store.c
  功能        : Remote DAQ 的板载存储与下载协议。
                采集结果分别以 CSV 文本和原始 RGB565 数据的形式写入外部
                SDRAM 的四组区域，ST-LINK 虚拟串口上的简易 ASCII 协议让
                PC 列出、取回并清空这些文件。文件标志字先清除、最后写入，
                因此 PC 不会读到写了一半的文件。

  说明        : 两条链路以及存储的每个字节都在此处理，
                显示部分只调用 Store_Init / Store_Save / Store_Poll。
  平台        : STM32F746G-DISCOVERY + STemWin (emWin 5.28) + IAR EWARM
******************************************************************************
*/

#include <stdint.h>
#include "GUI.h"
#include "LCD.h"
#include "DAQ_Link.h"
#include "RemoteDAQ_Private.h"

/******************************************************************************
*  上位机链路：经 ST-LINK 虚拟串口收发 ASCII 应答与数据
******************************************************************************/
#define HOST_BUF_SIZE  240

static char acHost[HOST_BUF_SIZE];
static int  hostLen;

/* 发送当前缓存的内容并清空缓冲。应答拼装完成时调用，
   缓冲即将溢出时也会调用。                                                  */
static void Host_Flush(void)
{
  if (hostLen)
  {
    Host_Put((const uint8_t *)acHost, (uint16_t)hostLen);
    hostLen = 0;
  }
}

/* 追加一个字符，空间不足时先刷出缓冲。 */
static void Host_Char(char c)
{
  acHost[hostLen++] = c;
  if (hostLen >= HOST_BUF_SIZE)
  {
    Host_Flush();
  }
}

/* 向主机应答中追加整个字符串。 */
static void Host_Str(const char *s)
{
  while (*s) Host_Char(*s++);
}

/* 以十进制追加无符号数，供命令应答使用。 */
static void Host_Num(unsigned v)
{
  char t[8];
  int  n = 0;

  do
  {
    t[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v);

  while (n) Host_Char(t[--n]);
}

/******************************************************************************
*  板载存储：外部 SDRAM 中的四组区域。每组保存同一次采集的 CSV 和 BMP，
*  即 csv1 + bmp1 属于第 1 组。
*
*    第 i 组 :  0xC06C0000 + i * 0x44000
*                 + 0x00000 : CSV 头 + CSV 文本        (16 kB)
*                 + 0x04000 : BMP 头 + RGB565 原始数据 (256 kB)
*
*  emWin 堆结束于 0xC06A0000，SDRAM 结束于 0xC0800000，因此
*  4 x 0x44000 = 1.06 MB 的整块空间可用。内容掉电丢失。
******************************************************************************/
#define STORE_BASE      0xC06C0000u
#define GRP_CSV_SIZE    0x04000u        /* 16 kB  : 400 行文本               */
#define GRP_BMP_SIZE    0x40000u        /* 256 kB : 480 x 272 x 2 字节       */
#define GRP_SIZE        (GRP_CSV_SIZE + GRP_BMP_SIZE)
#define STORE_HDR_SIZE  32u

#define ST_MAGIC        0x31444151u     /* "QAD1" */

typedef struct
{
  uint32_t magic;
  uint32_t type;
  uint32_t len;
  uint32_t tbUs;
  uint32_t reserved[4];
} STORE_HDR;

/* 某组的首字节：CSV 部分从这里开始，BMP 部分紧随 CSV 槽之后。              */
static uint8_t *Store_Base(uint32_t g)
{
  return (uint8_t *)(STORE_BASE + g * GRP_SIZE);
}

/* 某组中 CSV 或 BMP 部分的头部 */
static STORE_HDR *Store_Head(uint32_t g, uint32_t type)
{
  return (STORE_HDR *)(Store_Base(g) + ((type == ST_CSV) ? 0u : GRP_CSV_SIZE));
}

/* 一个文件的有效数据，即槽位地址加上小型存储头。 */
static uint8_t *Store_Data(uint32_t g, uint32_t type)
{
  return (uint8_t *)Store_Head(g, type) + STORE_HDR_SIZE;
}

/* 一个文件有效数据实际可用的字节数。 */
static uint32_t Store_Cap(uint32_t type)
{
  return ((type == ST_CSV) ? GRP_CSV_SIZE : GRP_BMP_SIZE) - STORE_HDR_SIZE;
}

/******************************************************************************
*  参考波形：每个组在保存的同时留下屏上那 400 个点的二进制副本。
*  CSV 只是文本，叠加显示时逐行解析代价太大，所以在 RAM 中直接留一份。
******************************************************************************/
static SCOPE_SAMPLE aRefSnap[STORE_GROUPS][N_DISP];
static uint8_t      aRefOk[STORE_GROUPS];      /* 1 = 该组已有快照 */

/* 把当前显示窗口的 400 个点复制为第 g 组的参考波形。 */
static void Store_Snapshot(uint32_t g)
{
  int i, idx, len;

  if (g >= STORE_GROUPS)
  {
    return;
  }

  len = App_BufLen();
  idx = App_StartIndex();
  for (i = 0; i < N_DISP; i++)
  {
    aRefSnap[g][i] = aSamples[idx];
    idx++;
    if (idx >= len)
    {
      idx = 0;
    }
  }
  aRefOk[g] = 1;
}

/* 第 g 组保存下来的参考波形，该组还没有存过任何文件时返回 0。 */
const SCOPE_SAMPLE *Store_Ref(uint32_t g)
{
  if (g >= STORE_GROUPS)
  {
    return 0;
  }
  return aRefOk[g] ? aRefSnap[g] : 0;
}

/* 启动时把所有槽位标记为无效，这样真正保存之前 PC 看到的是空存储。        */
void Store_Init(void)
{
  uint32_t g;

  for (g = 0; g < STORE_GROUPS; g++)
  {
    Store_Head(g, ST_CSV)->magic = 0;
    Store_Head(g, ST_BMP)->magic = 0;
    aRefOk[g] = 0;
  }
  storeGroup = 0;
}

/* 内存写入器，用于直接在槽位内拼出 CSV */
static uint8_t *poBuf;
static uint32_t poLen;
static uint32_t poCap;

/* 向输出槽追加一个字节，槽满时静默丢弃，调用者无需检查容量。 */
static void Out_Char(char c)
{
  if (poLen < poCap)
  {
    poBuf[poLen++] = (uint8_t)c;
  }
}

/* 向输出槽追加字符串。 */
static void Out_Str(const char *s)
{
  while (*s) Out_Char(*s++);
}

/* 向输出槽追加无符号数。 */
static void Out_Num(unsigned v)
{
  char t[8];
  int  n = 0;

  do
  {
    t[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v);

  while (n) Out_Char(t[--n]);
}

/* CSV：把屏幕上显示的 400 个采样点按屏幕顺序导出 */
static uint32_t Store_BuildCsv(uint8_t *dst, uint32_t cap)
{
  int      i, idx, len;
  uint32_t sampleUs;

  poBuf = dst;
  poLen = 0;
  poCap = cap;

  len      = App_BufLen();
  idx      = App_StartIndex();
  sampleUs = aTbUs[timebaseIdx] / SPP;

  Out_Str("# Remote DAQ sample export\r\n");
  Out_Str("# TB_US=");   Out_Num(aTbUs[timebaseIdx]); Out_Str("\r\n");
  Out_Str("# DT_US=");   Out_Num(sampleUs);           Out_Str("\r\n");
  Out_Str("# PWM_HZ=");  Out_Num((unsigned)pwmFreq);
  Out_Str(" PWM_DUTY="); Out_Num((unsigned)pwmDuty); Out_Str("\r\n");
  Out_Str("idx,t_us,ch1_mv,ch2_mv,D0,D1,D2\r\n");

  for (i = 0; i < N_DISP; i++)
  {
    Out_Num((unsigned)i);                          Out_Char(',');
    Out_Num((unsigned)((uint32_t)i * sampleUs));   Out_Char(',');
    Out_Num(aSamples[idx].ch1);                    Out_Char(',');
    Out_Num(aSamples[idx].ch2);                    Out_Char(',');
    Out_Char((aSamples[idx].dig & 1u) ? '1' : '0'); Out_Char(',');
    Out_Char((aSamples[idx].dig & 2u) ? '1' : '0'); Out_Char(',');
    Out_Char((aSamples[idx].dig & 4u) ? '1' : '0');
    Out_Str("\r\n");

    idx++;
    if (idx >= len) idx = 0;
  }

  return poLen;
}

/* BMP：屏幕按 480x272 RGB565 原始帧存储，由 PC 脚本转换成
   24 bpp 的 .bmp，板子因此不必构造文件头 */
static uint32_t Store_BuildBmp(uint8_t *dst, uint32_t cap)
{
  uint16_t *pRgb565 = (uint16_t *)dst;
  int       x, y, r, g, b;
  GUI_COLOR color;

  if (cap < (uint32_t)(SCR_W * SCR_H * 2))
  {
    return 0;
  }

  for (y = 0; y < SCR_H; y++)
  {
    for (x = 0; x < SCR_W; x++)
    {
      /* emWin 颜色为 0x00BBGGRR：先拆分，再打包成 RGB565 */
      color = LCD_GetPixelColor(x, y);
      r = (color >>  0) & 0xFFu;
      g = (color >>  8) & 0xFFu;
      b = (color >> 16) & 0xFFu;
      *pRgb565++ = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    }
  }
  return (uint32_t)(SCR_W * SCR_H * 2);
}

/* 保存所选组的一个文件。标志字先清除、最后写入，因此 PC 轮询存储时
   不会读到写了一半的文件。                                                  */
void Store_Save(uint32_t type)
{
  uint32_t   group = storeGroup;
  STORE_HDR *pHead = Store_Head(group, type);
  uint8_t   *dst   = Store_Data(group, type);
  uint32_t   cap   = Store_Cap(type);
  uint32_t   len;

  pHead->magic = 0;                    /* 写入过程中保持无效 */

  len = (type == ST_CSV) ? Store_BuildCsv(dst, cap)
                         : Store_BuildBmp(dst, cap);
  if (len == 0)
  {
    return;
  }

  pHead->type  = type;
  pHead->len   = len;
  pHead->tbUs  = aTbUs[timebaseIdx];
  pHead->magic = ST_MAGIC;

  /* 同时留下这一屏的参考波形，供 REF 叠加比较 */
  Store_Snapshot(group);

  /* "CSV2 OK"：数字为组号，csv2 与 bmp2 属于同一组 */
  statusText[0] = 0;
  Str_App(statusText, (type == ST_CSV) ? "CSV" : "BMP");
  Str_Int(statusText, group + 1u);
  Str_App(statusText, " OK");

  gDaq.lastTick = HAL_GetTick();
  App_DrawContent();
}

/******************************************************************************
*  下载协议（命令为 ASCII 行，由 PC 脚本发送）
*    ?               -> "DAQ1 <组数>"
*    LST             -> 每个已存文件一行 "GRP <组> <类型> <长度>"，最后 "END"
*    GET <组> <类型> -> "FILE <组> <类型> <长度>"、数据，最后 "END"
*    CLR             -> 清空所有组，应答 "OK"
*  <组> = 1..4，<类型> = 1 (CSV) / 2 (BMP)，也接受 'C' 和 'B'。
******************************************************************************/

/* 处理 LST 命令：每个组的每个有效文件输出一行。 */
static void Store_List(void)
{
  uint32_t g, t;

  hostLen = 0;
  for (g = 0; g < STORE_GROUPS; g++)
  {
    for (t = 0; t < 2u; t++)
    {
      uint32_t   type  = t ? ST_BMP : ST_CSV;
      STORE_HDR *pHead = Store_Head(g, type);

      if (pHead->magic != ST_MAGIC)
      {
        continue;
      }
      Host_Str("GRP "); Host_Num(g + 1u);
      Host_Char(' ');   Host_Num(type);
      Host_Char(' ');   Host_Num(pHead->len);
      Host_Str("\r\n");
    }
  }
  Host_Str("END\r\n");
  Host_Flush();
}

/* 处理 GET 命令：先发头行，再发数据，最后发结束标记。
   数据分块发送以缩短单次传输。                                              */
static void Store_SendFile(uint32_t g, uint32_t type)
{
  STORE_HDR *pHead = Store_Head(g, type);
  uint8_t   *pData = Store_Data(g, type);
  uint32_t   off;

  hostLen = 0;
  Host_Str("FILE "); Host_Num(g + 1u);
  Host_Char(' ');    Host_Num(type);
  Host_Char(' ');    Host_Num(pHead->len);
  Host_Str("\r\n");
  Host_Flush();

  for (off = 0; off < pHead->len; off += 4096u)
  {
    uint32_t chunkLen = pHead->len - off;
    if (chunkLen > 4096u)
    {
      chunkLen = 4096u;
    }
    Host_Put(pData + off, (uint16_t)chunkLen);
  }

  hostLen = 0;
  Host_Str("\r\nEND\r\n");
  Host_Flush();

  /* 传输会阻塞主循环，避免看起来像链路断开 */
  gDaq.lastTick = HAL_GetTick();
}

/* 解析并执行一条完整命令行。无法识别的命令直接忽略，
   因此杂散字节不会破坏协议。                                                */
static void Store_Cmd(const char *cmd)
{
  uint32_t pos;
  int      v, type;

  if ((cmd[0] == '?') && (cmd[1] == 0))
  {
    hostLen = 0;
    Host_Str("DAQ1 "); Host_Num(STORE_GROUPS);
    Host_Str("\r\n");
    Host_Flush();
  }
  else if ((cmd[0] == 'L') && (cmd[1] == 'S') && (cmd[2] == 'T'))
  {
    Store_List();
  }
  else if ((cmd[0] == 'C') && (cmd[1] == 'L') && (cmd[2] == 'R'))
  {
    Store_Init();
    hostLen = 0;
    Host_Str("OK\r\n");
    Host_Flush();
  }
  else if ((cmd[0] == 'G') && (cmd[1] == 'E') && (cmd[2] == 'T'))
  {
    pos = 3;
    while ((cmd[pos] == ' ') || (cmd[pos] == '\t')) pos++;

    v = 0;
    while ((cmd[pos] >= '0') && (cmd[pos] <= '9'))
    {
      v = v * 10 + (cmd[pos] - '0');
      pos++;
    }
    while ((cmd[pos] == ' ') || (cmd[pos] == '\t')) pos++;

    type = 0;
    if ((cmd[pos] == 'C') || (cmd[pos] == 'c') || (cmd[pos] == '1')) type = (int)ST_CSV;
    if ((cmd[pos] == 'B') || (cmd[pos] == 'b') || (cmd[pos] == '2')) type = (int)ST_BMP;

    if ((v >= 1) && (v <= (int)STORE_GROUPS) && (type != 0) &&
        (Store_Head((uint32_t)(v - 1), (uint32_t)type)->magic == ST_MAGIC))
    {
      Store_SendFile((uint32_t)(v - 1), (uint32_t)type);
    }
    else
    {
      hostLen = 0;
      Host_Str("ERR\r\n");
      Host_Flush();
    }
  }
}

/* 从主机链路收字符，每遇到行尾就执行一条命令。行长受限，
   超长行被截断，而不是溢出缓冲。                                            */
void Store_Poll(void)
{
  static char acCmd[16];
  static int  nCmd;
  int c;

  while ((c = Host_GetChar()) >= 0)
  {
    if ((c == '\r') || (c == '\n'))
    {
      if (nCmd)
      {
        acCmd[nCmd] = 0;
        Store_Cmd(acCmd);
        nCmd = 0;
      }
    }
    else if (nCmd < 15)
    {
      acCmd[nCmd++] = (char)c;
    }
  }
}

/*************************** 文件结束 ****************************/
