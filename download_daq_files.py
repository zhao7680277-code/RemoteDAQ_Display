#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Remote DAQ 显示板上保存的文件取回到 PC。

板载存储   ：外部 SDRAM 中划分 4 组，每组存放同一次采集的 CSV 和 BMP
             （csv1 与 bmp1 同属第 1 组）
  类型 1 = CSV（400 个采样点，即当时屏幕上显示的那一屏）
  类型 2 = BMP（屏幕的原始 RGB565 快照，480 x 272）

物理链路   ：ST-LINK 虚拟串口（连接显示板 USART1），921600 8N1

下载协议   ：纯 ASCII 行命令
  "?"              -> "DAQ1 <组数>"
  "LST"            -> "GRP <组> <类型> <长度>" ... "END"
  "GET <组> <类型>" -> "FILE <组> <类型> <长度>" + <长度> 字节 + "\\r\\nEND\\r\\n"
  "CLR"            -> "OK"

用法：
  python download_daq_files.py COM5
  python download_daq_files.py COM5 -o files
  python download_daq_files.py COM5 --list
  python download_daq_files.py COM5 --clear

依赖：pip install pyserial
"""

import argparse
import os
import struct
import sys
import time

try:
    import serial                       # pyserial
except ImportError:
    print("pyserial is missing, install it with:  pip install pyserial")
    sys.exit(2)

SCR_W = 480
SCR_H = 272
BMP_BYTES = SCR_W * SCR_H * 2

TYPE_CSV = 1
TYPE_BMP = 2


# --------------------------------------------------------------------------
#  串口收发辅助
# --------------------------------------------------------------------------
# 发送一条命令，协议以 CRLF 结尾
def send_line(ser, cmd):
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


# 读一行应答；超时（没有收到任何字节）视为板子无响应
def read_line(ser):
    line = ser.readline()
    if not line:
        raise TimeoutError("no answer from the board")
    return line.decode("ascii", "replace").strip()


# 收满指定字节数。文件数据是定长的，中途断流就报错，
# 避免把半截数据当成完整文件存下来。
def read_exact(ser, count):
    data = bytearray()
    while len(data) < count:
        chunk = ser.read(count - len(data))
        if not chunk:
            raise TimeoutError(
                "transfer interrupted : %d of %d bytes received" % (len(data), count))
        data += chunk
    return bytes(data)


# --------------------------------------------------------------------------
#  RGB565 -> 24 位 BMP
# --------------------------------------------------------------------------
# 板子只发屏幕像素（从上到下），BMP 文件头在这里补全
def rgb565_to_bmp(raw):
    """raw 为从上到下的 RGB565 像素，返回完整的 .bmp 文件内容。"""
    if len(raw) != BMP_BYTES:
        raise ValueError("bad snapshot size %d, expected %d" % (len(raw), BMP_BYTES))

    rows = bytearray()
    for y in range(SCR_H - 1, -1, -1):          # BMP 按从下到上的顺序存行
        base = y * SCR_W * 2
        for x in range(SCR_W):
            v = raw[base] | (raw[base + 1] << 8)
            base += 2
            r5 = (v >> 11) & 0x1F
            g6 = (v >> 5) & 0x3F
            b5 = v & 0x1F
            r8 = (r5 << 3) | (r5 >> 2)          # 5 位 -> 8 位
            g8 = (g6 << 2) | (g6 >> 4)          # 6 位 -> 8 位
            b8 = (b5 << 3) | (b5 >> 2)
            rows += bytes((b8, g8, r8))         # BMP 的字节顺序为 B G R

    offset = 54
    header = struct.pack("<2sIHHI", b"BM", offset + len(rows), 0, 0, offset)
    info = struct.pack("<IiiHHIIiiII", 40, SCR_W, SCR_H, 1, 24, 0,
                       len(rows), 2835, 2835, 0, 0)
    return header + info + bytes(rows)


# --------------------------------------------------------------------------
#  板端命令
# --------------------------------------------------------------------------
# 握手：重复发 "?" 直到板子回 "DAQ1 ..."，用于确认端口与板子都正常
def board_hello(ser, tries=8):
    for _ in range(tries):
        send_line(ser, "?")
        try:
            line = read_line(ser)
        except TimeoutError:
            continue
        if line.startswith("DAQ1"):
            return line
    raise TimeoutError("the board does not answer, check the COM port")


# 列出板子上已保存的文件，返回 (组, 类型, 长度) 列表，组号从 1 开始
def board_list(ser):
    send_line(ser, "LST")
    files = []
    while True:
        line = read_line(ser)
        if line == "END":
            return files
        parts = line.split()
        if parts and parts[0] == "GRP" and len(parts) >= 4:
            files.append((int(parts[1]), int(parts[2]), int(parts[3])))


# 取回一个文件：先读文件头行，再收定长数据，最后等到结束标记
def board_get(ser, group, ftype):
    send_line(ser, "GET %d %d" % (group, ftype))
    head = read_line(ser)
    if head.startswith("ERR"):
        raise RuntimeError("group %d type %d is empty" % (group, ftype))

    parts = head.split()
    if parts[0] != "FILE" or len(parts) < 4:
        raise RuntimeError("unexpected answer : %s" % head)

    got_group = int(parts[1])
    got_type = int(parts[2])
    length = int(parts[3])

    data = read_exact(ser, length)

    while True:                                # 数据之后是 "\r\nEND\r\n"
        tail = read_line(ser)
        if tail == "END":
            break
        if tail:
            raise RuntimeError("unexpected text after the payload : %s" % tail)

    return got_group, got_type, data


# --------------------------------------------------------------------------
#  主流程
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Download the files stored on the Remote DAQ board")
    ap.add_argument("port", help="serial port of the ST-LINK, e.g. COM5")
    ap.add_argument("-b", "--baud", type=int, default=921600,
                    help="must match the board, default 921600")
    ap.add_argument("-o", "--out", default=None,
                    help="output directory, default <project>/saved")
    ap.add_argument("--list", dest="list_only", action="store_true",
                    help="only show the stored files, do not download")
    ap.add_argument("--clear", action="store_true",
                    help="erase every group on the board after the download")
    args = ap.parse_args()

    # 默认输出到脚本所在目录下的 saved，与当前工作目录无关
    if not args.out:
        args.out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as exc:
        print("cannot open %s : %s" % (args.port, exc))
        return 2

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.2)

    try:
        print("board : %s" % board_hello(ser))
        ser.timeout = 10                    # 一整幅截图的传输需要几秒钟

        files = board_list(ser)
        if not files:
            print("no file stored on the board, press SAVE CSV / SAVE BMP first")
            return 0

        print("stored files :")
        for group, ftype, length in files:
            name = "CSV" if ftype == TYPE_CSV else "BMP"
            print("  group %d  %-3s  %d bytes" % (group, name, length))

        if args.list_only:
            return 0

        os.makedirs(args.out, exist_ok=True)
        total = 0

        for group, ftype, length in files:
            print("downloading group %d %s (%d bytes) ..."
                  % (group, "CSV" if ftype == TYPE_CSV else "BMP", length))
            got_group, got_type, data = board_get(ser, group, ftype)

            if got_type == TYPE_CSV:
                path = os.path.join(args.out, "daq_group%d.csv" % got_group)
                with open(path, "wb") as f:
                    f.write(data)
            elif got_type == TYPE_BMP:
                path = os.path.join(args.out, "daq_group%d.bmp" % got_group)
                with open(path, "wb") as f:
                    f.write(rgb565_to_bmp(data))
            else:
                path = os.path.join(args.out, "daq_group%d.bin" % got_group)
                with open(path, "wb") as f:
                    f.write(data)

            total += 1
            print("  -> %s (%d bytes)" % (path, len(data)))

        if args.clear:
            send_line(ser, "CLR")
            print("board : %s" % read_line(ser))

        print("done, %d file(s) written to %s" % (total, os.path.abspath(args.out)))
        return 0
    except (TimeoutError, RuntimeError, ValueError) as exc:
        print("error : %s" % exc)
        return 1
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
