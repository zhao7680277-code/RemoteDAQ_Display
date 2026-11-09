远程数据采集系统 - 显示端 (RemoteDAQ_Display)
================================================================

一、工程说明
  本工程为课程设计《远程数据采集系统》的显示端程序，运行于
  STM32F746G-DISCOVERY 开发板，使用 SEGGER STemWin 图形库绘制
  示波器风格界面：2 路模拟通道 + 3 路数字通道，8 格 x 50 点。

二、开发环境
  IAR Embedded Workbench for ARM 8.2x
  打开 EWARM\Project.eww 即可（工程已在包内自带 Drivers、Middlewares，
  不依赖外部 STM32Cube 包，可直接编译）

三、数据链路
  显示端 USART6 : PC6 = TX (Arduino D1), PC7 = RX (Arduino D0)
                  4000000 bps, 8N1, 无流控, DMA2 Stream1 循环接收
  上位机    USART1 : PA9 = TX, PB7 = RX (ST-LINK 虚拟串口, 用于下载文件)
                  921600 bps, 8N1, 无流控

四、通信协议（上行数据帧格式说明 v1.0）
  帧长 1770 字节：AA 55 | VER=01 | TYPE=01 | SEQ(4B 小端) | LEN=1758 |
  参数区(8B) | CH1 400xuint16 | CH2 400xuint16 | 数字量 3x50B 位打包 |
  CRC16-CCITT-FALSE(2B)，校验范围 frameBuf[2..1767]
  控制帧（显示端 -> 采集端，ASCII）：A<时基档1..5>B<频率>C<占空比>D

五、源码结构
  STemWin\App\DAQ_Link.c / .h    串口收发、帧同步、CRC 校验、参数解析
  STemWin\App\RemoteDAQ.c        主流程：主循环、按键动作、触摸处理
  STemWin\App\DAQ_Draw.c         界面绘制：顶栏、按钮、网格、波形、测量行
  STemWin\App\DAQ_Sim.c          采样环形缓冲、窗口起点与触发索引、仿真波形
  STemWin\App\DAQ_Measure.c      峰峰值、频率、中心电平、脉宽测量
  STemWin\App\DAQ_Store.c        CSV / BMP 存储与上位机下载协议
  STemWin\App\DAQ_Util.c         无 printf 的整数/电压/频率格式化
  STemWin\App\RemoteDAQ_Private.h  模块间共享的布局、颜色、类型与外部声明
  STemWin\Target\LCDConf.c       LTDC + SDRAM 显存配置
  STemWin\Target\GUIConf.c       STemWin 堆配置
  Core\Src\main.c                HAL 初始化与主循环入口 (MainTask)

六、功能
  1. 5 档时基：10ms / 5ms / 1ms / 500us / 100us 每格，可按 T- / T+ 切换
  2. 触发：Auto / Normal / Single，触发源 CH1、CH2、D0~D2，触发电平可调，
     电平位置在绘图区用暗橙色虚线标出（数字源在对应行标 T）；触发点固定
     放在窗口左侧 1/5 处，触发沿前后的波形都能看到
       Auto   找不到触发点时退回波形中点，画面始终有波形
       Normal 本帧没有触发就保持上一帧，波形停在最后一次触发上
       Single 抓到一次触发后停住，按 RUN 重新武装
  3. 垂直灵敏度：660 / 330 / 165 / 66 / 33 mV/div，切换时自动纵向居中
  4. 测量：峰峰值、频率、中心电平、数字通道频率与正脉宽
  5. 存储：CSV 数据导出、BMP 截图保存到板载 SDRAM，再经 USART1 下载
  6. 无信号时自动切换为内置仿真波形，接收正常后自动回到实时模式
  7. 波形比较（P4 页 REF 键）：把某一组保存时的 400 点波形用暗色叠加到
     当前波形下，可循环选择第 1~4 组或关闭，用于前后两次采集对比
  8. 光标测量（P5 页）：绘图区出现 2 条竖线和 2 条横线，可按住拖动，
     也可用按键精确微调——SEL 依次切换 X1/X2/Y1/Y2，< > 左右挪竖线，
     ^ v 上下挪横线，每按一次 1 个像素；被选中的那条线用亮白色显示。
     屏幕左上角实时显示两条竖线的时间差 dt 与 1/dt（等效频率）、
     两条横线的电压差 dv

七、依赖
  Drivers      STM32F7xx HAL / CMSIS / BSP (ST 官方，原样使用)
  Middlewares  STemWin 5.28 (STemWin_CM7_wc32.a，ST 官方库)
  Utilities    字体目录（预留）

八、页面
  P1  波形页：CH1/CH2/D0/D1/D2 显示开关、T- / T+、RUN
  P2  信号源页：PWM 频率与占空比、RUN、垂直灵敏度
  P3  触发页：触发方式、触发源、触发电平、RUN
  P4  存储页：分组、SAVE CSV、SAVE BMP、REF 参考波形
  P5  光标页：CUR 开关、SEL 选线、四方向微调
