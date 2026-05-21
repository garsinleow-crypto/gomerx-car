#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
夹爪抓取测试
测试 catch.py 模块的抓取动作

用法:
  python test_catch.py
"""
import sys
import time
from catch import catch

ROBOT_NAME = 'GomerX_Y6HFw4'


def main():
    print('=' * 50)
    print('[测试] 夹爪抓取')
    print('=' * 50)
    print()
    print('预期动作:')
    print('  1. 夹爪打开')
    print('  2. 机械臂移动到抓取位 (18, 9)')
    print('  3. 夹爪闭合')
    print('  4. 机械臂抬起到 (18, 19)')
    print('  5. 机械臂移动到避障位 (13, 19)')
    print()
    print('请确认试管已放置在抓取位置...')
    print()

    # 等待确认
    if len(sys.argv) < 2:
        confirm = input('按回车开始测试 (Ctrl+C取消): ')
    else:
        confirm = sys.argv[1]

    print()
    print('[开始] 执行抓取...')
    print()

    try:
        catch(robot_name=ROBOT_NAME)
        print()
        print('[完成] 抓取动作执行完毕')
        print('=' * 50)
    except KeyboardInterrupt:
        print()
        print('[取消] 测试已取消')
        sys.exit(0)
    except Exception as e:
        print()
        print(f'[!] 错误: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
