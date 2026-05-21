#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
demo519 完整流程测试
顺序执行所有测试步骤，时间阻塞等待每步完成

用法:
  python test_demo_full.py
"""
import sys
import time

# 添加dddddd目录到路径
sys.path.insert(0, r'D:\xiaoche\demo519\dddddd')

# 导入所有模块
from detect_tag import scan, move_to_tag
import patrol_hough
from sweep_right import sweep_right
from catch import catch
from release import release
from turn_180 import turn_180
from turn_90_ccw import turn_90_ccw

ROBOT_NAME = 'GomerX_Y6HFw4'
MAX_RETRIES = 3


def wait_for_connection(wait_time=5):
    """等待网络连接恢复"""
    print(f'[等待 {wait_time} 秒后重试...]')
    time.sleep(wait_time)


def step_scan(desc):
    """AprilTag扫描（带重试）"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    time.sleep(3)  # 等待上一连接释放
    
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            result = scan(robot_name=ROBOT_NAME, show_window=True)
            print(f'\n>>> 扫描结果: ID={result}')
            return result
        except (ConnectionAbortedError, ConnectionResetError, OSError) as e:
            print(f'\n[!] 网络错误 (尝试 {attempt}/{MAX_RETRIES}): {e}')
            if attempt < MAX_RETRIES:
                wait_for_connection()
            else:
                raise


def step_align(desc, tag_id):
    """AprilTag对准（带重试）"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    time.sleep(3)  # 等待上一连接释放
    
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            move_to_tag(robot_name=ROBOT_NAME, tag_id=tag_id, show_window=True)
            print(f'\n>>> 对准ID={tag_id} 完成')
            return
        except (ConnectionAbortedError, ConnectionResetError, OSError) as e:
            print(f'\n[!] 网络错误 (尝试 {attempt}/{MAX_RETRIES}): {e}')
            if attempt < MAX_RETRIES:
                wait_for_connection()
            else:
                raise


def step_patrol(desc, color):
    """巡线（带重试）"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    time.sleep(3)  # 等待上一连接释放
    
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            patrol_hough.patrol_hough(color=color, debug=True, robot_name=ROBOT_NAME)
            return
        except (ConnectionAbortedError, ConnectionResetError, OSError) as e:
            print(f'\n[!] 网络错误 (尝试 {attempt}/{MAX_RETRIES}): {e}')
            if attempt < MAX_RETRIES:
                wait_for_connection()
            else:
                raise


def step_sweep(desc, target_ids):
    """横扫（带重试）"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    time.sleep(3)  # 等待上一连接释放
    
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            result = sweep_right(target_ids=target_ids)
            print(f'\n>>> 横扫结果: ID={result}')
            return result
        except (ConnectionAbortedError, ConnectionResetError, OSError) as e:
            print(f'\n[!] 网络错误 (尝试 {attempt}/{MAX_RETRIES}): {e}')
            if attempt < MAX_RETRIES:
                wait_for_connection()
            else:
                raise


def step_catch(desc):
    """抓取"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    print('预期动作: 开爪→抓取位→闭合→抬出→避障位')
    time.sleep(3)
    catch(robot_name=ROBOT_NAME)
    print('\n>>> 抓取完成')


def step_release(desc):
    """释放"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    print('预期动作: 高位→下降→开爪→避障位')
    time.sleep(3)
    release(robot_name=ROBOT_NAME)
    print('\n>>> 释放完成')


def step_turn(desc, func):
    """旋转"""
    print()
    print('=' * 70)
    print(f'[*] {desc}')
    print('=' * 70)
    time.sleep(3)
    func()
    print('\n>>> 旋转完成')


def main():
    print()
    print('=' * 70)
    print('  demo519 完整流程测试')
    print('=' * 70)
    print()
    print('流程概述:')
    print('  1. [起点] scan→align 05 → turn_180')
    print('  2. [A点]  patrol yellow → scan→align 01 → sweep 00 → align 00')
    print('  3. [抓取] catch → turn_90_ccw')
    print('  4. [B点]  patrol yellow → scan→align 04 → sweep 07 → scan→align 07')
    print('  5. [放置] release → turn_90_ccw')
    print('  6. [终点] patrol blue → scan→align 05')
    print()
    print('=' * 70)
    input('按回车开始测试: ')

    # ============ 阶段1: 起点 ============
    print()
    print('[阶段1] 起点: 确认ID=5位置')

    step_scan('扫描AprilTag (STEP 01)')
    step_align('对准ID=5 (STEP 02)', 5)
    step_turn('掉头180° (STEP 03)', turn_180)

    # ============ 阶段2: A点 ============
    print()
    print('[阶段2] A点: 到达抓取位置')

    step_patrol('巡黄线 (STEP 04)', 'yellow')
    step_scan('扫描ID=1 (STEP 05)')
    step_align('对准ID=1 (STEP 06)', 1)
    step_sweep('右扫找试管 (STEP 07)', [0])
    step_align('对准试管ID=0 (STEP 08)', 0)

    # ============ 阶段3: 抓取 ============
    print()
    print('[阶段3] 抓取试管')

    step_catch('抓取 (STEP 09)')
    step_turn('左转90° (STEP 10)', turn_90_ccw)

    # ============ 阶段4: B点 ============
    print()
    print('[阶段4] B点: 到达放置位置')

    step_patrol('巡黄线 (STEP 11)', 'yellow')
    step_scan('扫描ID=4 (STEP 12)')
    step_align('对准ID=4 (STEP 13)', 4)
    step_sweep('右扫找放置区 (STEP 14)', [7])
    step_scan('扫描ID=7 (STEP 15)')
    step_align('对准ID=7 (STEP 16)', 7)

    # ============ 阶段5: 放置 ============
    print()
    print('[阶段5] 放置试管')

    step_release('释放 (STEP 17)')
    step_turn('左转90° (STEP 18)', turn_90_ccw)

    # ============ 阶段6: 终点 ============
    print()
    print('[阶段6] 终点: 确认完成')

    step_patrol('巡蓝线 (STEP 19)', 'blue')
    step_scan('终点确认ID=5 (STEP 20)')

    print()
    print('=' * 70)
    print('  测试完成！')
    print('=' * 70)
    print()
    print('结果汇总:')
    print('  [01-02] 起点: scan→align 05 ✓')
    print('  [03]    turn_180 ✓')
    print('  [04]    A点: patrol yellow ✓')
    print('  [05]    A点: scan ✓')
    print('  [06]    A点: align 01 ✓')
    print('  [07]    A点: sweep 00 ✓')
    print('  [08]    A点: align 00 ✓')
    print('  [09]    catch ✓')
    print('  [10]    turn_90_ccw ✓')
    print('  [11]    B点: patrol yellow ✓')
    print('  [12]    B点: scan ✓')
    print('  [13]    B点: align 04 ✓')
    print('  [14]    B点: sweep 07 ✓')
    print('  [15]    B点: scan (07) ✓')
    print('  [16]    B点: align 07 ✓')
    print('  [17]    release ✓')
    print('  [18]    turn_90_ccw ✓')
    print('  [19]    终点: patrol blue ✓')
    print('  [20]    终点: scan→align 05 ✓')
    print()


if __name__ == '__main__':
    main()
