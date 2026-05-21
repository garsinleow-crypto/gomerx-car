"""
nav_action.py - 根据扫描ID执行对应动作
单次连接模式，避免频繁断开/重连导致摄像头冲突

函数: nav_action(tag_id, last_id=None, robot_name=ROBOT_NAME)
  tag_id:   当前扫描返回的ID（动作前扫描）
  last_id:   上一个扫描返回的ID
  return:    当前动作完成后扫描返回的ID

业务逻辑:
  id=10  -> C初始化:     move_to_tag
  id=1   -> Astart:       move_to_tag -> 右横移扫描返回id
  id=2   -> Bstart:       move_to_tag -> 左横移扫描返回id
  id=3   -> Aend:         逆时针90度 -> 巡线 -> 终点扫描返回id
  id=4   -> Bend:         顺时针90度 -> 巡线 -> 终点扫描返回id
  id=0+last=1 -> 试管:   move_to_tag -> catch -> 顺时针90 -> 巡线 -> 终点扫描
  id=0+last=2 -> 试管:   move_to_tag -> catch -> 逆时针90 -> 巡线 -> 终点扫描
"""
import time
import cv2 as cv
import numpy as np
import pupil_apriltags as at
from collections import Counter
from gomerx import robot
from gomerx.skill import LINE_END

# ---- 参数 ----
ROBOT_NAME    = 'GomerX_Y6HFw4'
TAG_FAMILY    = 'tag36h11'
TAG_SIZE_M    = 0.02
CAMERA_PARAMS = (650.0, 650.0, 400.0, 300.0)
TARGET_DIST_CM = 23.5
TOL_CM        = 0.5
MAX_ITER      = 50
CHASSIS_MIN   = 1.0
SWEEP_STEP    = 5
SWEEP_TIMEOUT = 30
SCAN_COUNT    = 15
SCAN_INTERVAL = 0.4
HSV_LOW       = (24, 16, 67)
HSV_HIGH      = (44, 56, 100)

# ---- ID常量 ----
ID_CINIT  = 10
ID_ASTART = 1
ID_BSTART = 2
ID_AEND   = 3
ID_BEND   = 4
ID_TEST   = 0

NAME_MAP = {
    ID_CINIT:  'Cinit',
    ID_ASTART: 'Astart',
    ID_BSTART: 'Bstart',
    ID_AEND:   'Aend',
    ID_BEND:   'Bend',
    ID_TEST:   'Test',
}


# ============================================================
# 工具函数（内联，无独立连接）
# ============================================================

def _arm_ready(my_robot):
    """抬臂避障"""
    my_robot.arm.move_to(12, 15)
    time.sleep(0.2)


def _move_horizontal(my_robot, direction):
    """
    横向移动一步（替换rotate_360_search中的旋转）
    direction: 'left'(x=-5) 或 'right'(x=+5)
    """
    step = SWEEP_STEP if direction == 'right' else -SWEEP_STEP
    _arm_ready(my_robot)
    my_robot.chassis.move(x=step, y=0, a=0, wait_for_complete=True)
    time.sleep(0.2)


def _align_to_tag(my_robot, tag_id, show_window=True):
    """
    对准指定Tag到TARGET_DIST_CM距离
    返回: bool
    """
    detector = at.Detector(
        families=TAG_FAMILY, nthreads=2, quad_decimate=2.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
    )
    camera  = my_robot.camera
    chassis = my_robot.chassis

    camera.start_video_stream(display=False)
    time.sleep(1.5)

    window = 'Align'
    if show_window:
        cv.namedWindow(window, cv.WINDOW_AUTOSIZE)

    print(f'[*] [对准] Tag ID={tag_id} -> {TARGET_DIST_CM}cm')

    for i in range(MAX_ITER):
        frame = camera.read_cv_image()
        if frame is None:
            time.sleep(0.05)
            continue

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        tags = detector.detect(gray, estimate_tag_pose=True,
                              camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M)

        # 绘制
        for tag in tags:
            color = (0, 255, 0) if tag.tag_id == tag_id else (0, 128, 255)
            c = tag.corners.astype(int)
            cv.polylines(frame, [c.reshape((-1, 1, 2))],
                         isClosed=True, color=color, thickness=2)
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv.circle(frame, (cx, cy), 4, (0, 165, 255), -1)
            if tag.pose_t is not None:
                d = np.linalg.norm(tag.pose_t.flatten()) * 100
                cv.putText(frame, f'ID={tag.tag_id} {d:.0f}cm',
                          (cx - 30, cy - 10),
                          cv.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # 找目标
        has_pose = None
        no_pose  = None
        for tag in tags:
            if tag.tag_id == tag_id:
                if tag.pose_t is not None:
                    has_pose = tag; break
                elif no_pose is None:
                    no_pose = tag

        if has_pose is not None:
            target = has_pose
            t = target.pose_t.flatten()
            tz_cm = float(t[2]) * 100
            tx_cm = float(t[0]) * 100
            err_z = tz_cm - TARGET_DIST_CM
            print(f'[iter {i+1:02d}] tz={tz_cm:.1f}cm tx={tx_cm:+.1f}cm')
        elif no_pose is not None:
            fw   = frame.shape[1]
            cx_t = int(no_pose.center[0])
            tx_cm = (cx_t - fw / 2) / (fw / 2) * 50
            err_z = 30
            print(f'[iter {i+1:02d}] tz~30cm tx~{tx_cm:+.1f}cm')
        else:
            # 未找到，旋转寻找
            other = [t for t in tags if t.tag_id != tag_id]
            if other:
                fw  = frame.shape[1]
                off = (int(other[0].center[0]) - fw / 2) / (fw / 2)
                _arm_ready(my_robot)
                chassis.move(x=0, y=0, a=off * 30, wait_for_complete=True)
            else:
                _arm_ready(my_robot)
                chassis.move(x=0, y=-5, a=0, wait_for_complete=True)
            time.sleep(0.3)
            if show_window:
                cv.imshow(window, frame)
                cv.waitKey(1)
            continue

        # 判断是否到达
        if abs(err_z) < TOL_CM and abs(tx_cm) < TOL_CM:
            print('[+] [对准] 到达')
            camera.stop_video_stream()
            if show_window: cv.destroyWindow(window)
            return True

        move_x = max(-20, min(20, tx_cm)) if abs(tx_cm) > TOL_CM else 0.0
        move_y = max(-20, min(20, err_z)) if abs(err_z) > TOL_CM else 0.0
        has_x  = abs(move_x) >= CHASSIS_MIN
        has_y  = abs(move_y) >= CHASSIS_MIN

        if not (has_x or has_y):
            print('[+] [对准] 最小步长范围')
            camera.stop_video_stream()
            if show_window: cv.destroyWindow(window)
            return True

        _arm_ready(my_robot)
        chassis.move(x=move_x if has_x else 0,
                     y=move_y if has_y else 0,
                     a=0, wait_for_complete=True)
        time.sleep(0.3)

        if show_window:
            cv.imshow(window, frame)
            cv.waitKey(1)

    camera.stop_video_stream()
    if show_window: cv.destroyWindow(window)
    print('[!] [对准] 超时')
    return False


def _sweep_scan(my_robot, direction, exclude_ids=None, show_window=True):
    """
    横移扫描（替换rotate_360_search的逻辑）
    参考: myrobot_test.py中的rotate_360_search
    把skill.detect_pattern换成AprilTag检测
    把chassis.move(a=rotate_angle)换成chassis.move(x=step)

    参数:
        direction:    'left' 或 'right'
        exclude_ids:  排除的ID列表（不识别这些ID）
    返回:
        int or None: 扫描到的ID，超时返回None
    """
    if exclude_ids is None:
        exclude_ids = []

    detector = at.Detector(
        families=TAG_FAMILY, nthreads=2, quad_decimate=2.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
    )
    camera  = my_robot.camera
    chassis = my_robot.chassis

    camera.start_video_stream(display=False)
    time.sleep(1.0)

    window = 'Sweep'
    if show_window:
        cv.namedWindow(window, cv.WINDOW_AUTOSIZE)

    print(f'[*] [横扫] 方向={direction}, 排除={exclude_ids}')

    start_time  = time.time()
    sweep_count = 0

    # 参考rotate_360_search: 循环横移+检测
    while time.time() - start_time < SWEEP_TIMEOUT:
        # -- 横移动作（替换原来的旋转）--
        _move_horizontal(my_robot, direction)
        sweep_count += 1

        # -- 检测（替换skill.detect_pattern）--
        frame = camera.read_cv_image()
        if frame is None or frame.size == 0:
            time.sleep(0.1)
            continue

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        tags = detector.detect(gray, estimate_tag_pose=True,
                              camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M)

        # 绘制
        for tag in tags:
            if tag.tag_id in exclude_ids:
                color = (60, 60, 60)   # 灰色：排除
            else:
                color = (0, 255, 0)    # 绿色：目标
            c = tag.corners.astype(int)
            cv.polylines(frame, [c.reshape((-1, 1, 2))],
                         isClosed=True, color=color, thickness=2)
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv.circle(frame, (cx, cy), 4, (0, 165, 255), -1)
            if tag.pose_t is not None:
                d = np.linalg.norm(tag.pose_t.flatten()) * 100
                cv.putText(frame, f'ID={tag.tag_id} {d:.0f}cm',
                          (cx - 30, cy - 10),
                          cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if show_window:
            cv.imshow(window, frame)
            cv.waitKey(1)

        # -- 检查目标（排除指定ID）--
        for tag in tags:
            if tag.tag_id not in exclude_ids:
                # 找到目标！
                found_id = tag.tag_id
                print(f'[+] [横扫] 找到 ID={found_id} (横移{sweep_count}次)')
                camera.stop_video_stream()
                if show_window: cv.destroyWindow(window)
                return found_id

        if sweep_count % 5 == 0:
            print(f'[*] [横扫] 已横移 {sweep_count} 次...')

    # 超时
    print('[!] [横扫] 超时未找到目标')
    camera.stop_video_stream()
    if show_window: cv.destroyWindow(window)
    return None


def _catch(my_robot):
    """抓取"""
    print('[*] [抓取] 执行')
    arm     = my_robot.arm
    gripper = my_robot.gripper
    gripper.open()
    arm.move_to(18, 9)
    gripper.close()
    arm.move_to(18, 19)
    arm.move_to(13, 19)
    print('[+] [抓取] 完成')


def _rotate(my_robot, degree):
    """旋转：正=逆时针，负=顺时针"""
    if abs(degree) < 1:
        return
    direction = '逆时针' if degree > 0 else '顺时针'
    print(f'[*] [旋转] {direction} {abs(degree)}度')
    _arm_ready(my_robot)
    my_robot.chassis.move(x=0, y=0, a=degree, wait_for_complete=True)
    time.sleep(0.5)


def _patrol_and_scan(my_robot, show_window=True):
    """
    巡线 -> 到终点后扫描返回ID
    返回: int or None
    """
    skill  = my_robot.skill
    camera = my_robot.camera

    print('[*] [巡线] 开始')
    result, line_coords = skill.detect_line(
        hsv_low=HSV_LOW, hsv_high=HSV_HIGH, timeout=5,
    )
    if not result:
        print('[!] [巡线] 未检测到黄线')
        return None

    print(f'[*] [巡线] 坐标: {line_coords}')
    ok = skill.move_along_line(stop=LINE_END)
    if ok:
        print('[+] [巡线] 到达终点')
    else:
        print('[!] [巡线] 异常结束')

    # 到终点后扫描
    print('[*] [扫描] 终点扫描...')
    detector = at.Detector(
        families=TAG_FAMILY, nthreads=2, quad_decimate=2.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
    )
    camera.start_video_stream(display=False)
    time.sleep(1.0)

    id_counter = Counter()

    for i in range(SCAN_COUNT):
        frame = camera.read_cv_image()
        if frame is None:
            time.sleep(0.05)
            continue
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        tags = detector.detect(gray, estimate_tag_pose=True,
                              camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M)
        for tag in tags:
            id_counter[tag.tag_id] += 1
        time.sleep(SCAN_INTERVAL)

    camera.stop_video_stream()

    if id_counter:
        most_id = id_counter.most_common(1)[0][0]
        print(f'[+] [扫描] 返回 ID={most_id}')
        return most_id
    else:
        print('[!] [扫描] 未检测到Tag')
        return None


# ============================================================
# 主函数
# ============================================================

def nav_action(tag_id, last_id=None, robot_name=ROBOT_NAME, show_window=True):
    """
    根据Tag ID执行对应动作

    参数:
        tag_id:     当前扫描返回的ID（动作前）
        last_id:     上一个扫描返回的ID
        robot_name:  机器人名称
        show_window: 是否显示窗口
    返回:
        int or None: 动作完成后扫描返回的ID
    """
    name = NAME_MAP.get(tag_id, f'ID{tag_id}')
    print(f'\n{"="*50}')
    print(f'  nav_action: {name} (ID={tag_id}, last={last_id})')
    print(f'{"="*50}\n')

    my_robot  = robot.Robot(robot_name)
    result_id = None

    try:
        # -- ID=10: C初始化 --
        if tag_id == ID_CINIT:
            print('[*] [Cinit] 对准Tag10')
            _align_to_tag(my_robot, ID_CINIT, show_window=show_window)
            result_id = ID_CINIT
            print('[+] [Cinit] 完成')

        # -- ID=1: Astart --
        elif tag_id == ID_ASTART:
            print('[*] [Astart] 对准自身 -> 右横移扫描')
            _align_to_tag(my_robot, ID_ASTART, show_window=show_window)
            time.sleep(2.0)
            # 右横移扫描，排除ID=1,2
            result_id = _sweep_scan(my_robot, direction='right',
                                    exclude_ids=[ID_ASTART, ID_BSTART],
                                    show_window=show_window)
            if result_id:
                print(f'[+] [Astart] 横扫返回 ID={result_id}')
            else:
                print('[!] [Astart] 横扫未找到目标')

        # -- ID=2: Bstart --
        elif tag_id == ID_BSTART:
            print('[*] [Bstart] 对准自身 -> 左横移扫描')
            _align_to_tag(my_robot, ID_BSTART, show_window=show_window)
            time.sleep(2.0)
            # 左横移扫描，排除ID=1,2
            result_id = _sweep_scan(my_robot, direction='left',
                                    exclude_ids=[ID_ASTART, ID_BSTART],
                                    show_window=show_window)
            if result_id:
                print(f'[+] [Bstart] 横扫返回 ID={result_id}')
            else:
                print('[!] [Bstart] 横扫未找到目标')

        # -- ID=3: Aend --
        elif tag_id == ID_AEND:
            print('[*] [Aend] 逆时针90度 -> 巡线 -> 扫描')
            _rotate(my_robot, 90)   # 逆时针
            result_id = _patrol_and_scan(my_robot, show_window=show_window)

        # -- ID=4: Bend --
        elif tag_id == ID_BEND:
            print('[*] [Bend] 顺时针90度 -> 巡线 -> 扫描')
            _rotate(my_robot, -90)  # 顺时针
            result_id = _patrol_and_scan(my_robot, show_window=show_window)

        # -- ID=0: 测试试管 --
        elif tag_id == ID_TEST:
            if last_id == ID_ASTART:
                print('[*] [试管] last=Astart -> 对准 -> 抓取 -> 顺时针 -> 巡线')
                _align_to_tag(my_robot, ID_TEST, show_window=show_window)
                time.sleep(1.5)
                _catch(my_robot)
                _rotate(my_robot, 90)   # 顺时针
                result_id = _patrol_and_scan(my_robot, show_window=show_window)

            elif last_id == ID_BSTART:
                print('[*] [试管] last=Bstart -> 对准 -> 抓取 -> 逆时针 -> 巡线')
                _align_to_tag(my_robot, ID_TEST, show_window=show_window)
                time.sleep(1.5)
                _catch(my_robot)
                _rotate(my_robot, -90)  # 逆时针
                result_id = _patrol_and_scan(my_robot, show_window=show_window)

            else:
                print(f'[!] [试管] 未知last_id={last_id}，无法执行动作')
                result_id = None

        else:
            print(f'[!] [nav] 未知Tag ID: {tag_id}')
            result_id = None

    finally:
        my_robot.close()
        print(f'[*] [nav] 返回 ID={result_id}\n')

    return result_id


# ============================================================
# 测试入口
# ============================================================

if __name__ == '__main__':
    print(__doc__)
    print('\n用法示例:')
    print('  nav_action(10)     # C初始化')
    print('  nav_action(1)      # Astart')
    print('  nav_action(2)      # Bstart')
    print('  nav_action(3)      # Aend')
    print('  nav_action(4)      # Bend')
    print('  nav_action(0, 1)   # 试管+last=Astart')
    print('  nav_action(0, 2)   # 试管+last=Bstart')
