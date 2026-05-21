#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
AprilTag 识别定位测试工具
功能：
  1. scan   - 扫描识别，返回检测到的Tag ID
  2. align  - 对准定位，移动到指定Tag前23.5cm处
  3. quit   - 退出

用法:
  python test_apriltag.py [mode] [tag_id]
  示例:
    python test_apriltag.py scan       # 扫描识别
    python test_apriltag.py align 1   # 对准ID=1
    python test_apriltag.py            # 交互模式
"""
import sys
from detect_tag import scan, move_to_tag

ROBOT_NAME = 'GomerX_Y6HFw4'


def main():
    args = sys.argv[1:]

    if not args:
        # 交互模式
        print(__doc__)
        while True:
            cmd = input('\n请输入命令 (scan/align [id]/quit): ').strip()
            if not cmd:
                continue
            parts = cmd.split()
            if parts[0] == 'scan':
                print('\n[*] 扫描模式...')
                result = scan(robot_name=ROBOT_NAME, show_window=True)
                print(f'\n>>> 识别结果: ID={result}')
            elif parts[0] == 'align':
                tag_id = int(parts[1]) if len(parts) > 1 else 0
                print(f'\n[*] 对准模式，目标ID={tag_id}...')
                move_to_tag(robot_name=ROBOT_NAME, tag_id=tag_id, show_window=True)
            elif parts[0] == 'quit':
                print('[*] 退出')
                break
            else:
                print('[!] 未知命令，有效命令: scan, align [id], quit')

    elif len(args) == 1:
        cmd = args[0]
        if cmd == 'scan':
            result = scan(robot_name=ROBOT_NAME, show_window=True)
            print(f'\n>>> 识别结果: ID={result}')
        elif cmd == 'quit':
            pass
        else:
            print(f'[!] 未知命令: {cmd}')
            print('用法: python test_apriltag.py [scan|quit]')
    else:
        cmd, tag_id = args[0], int(args[1])
        if cmd == 'align':
            move_to_tag(robot_name=ROBOT_NAME, tag_id=tag_id, show_window=True)
        else:
            print('[!] 参数错误')
            print('用法: python test_apriltag.py [scan|align id|quit]')


if __name__ == '__main__':
    main()
