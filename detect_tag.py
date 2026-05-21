"""
AprilTag 检测/对准模块 - 只有def，无main
支持：扫描返回ID / 对准到指定距离
"""
import time
import cv2
import numpy as np
from gomerx import robot
from gomerx.skill import LINE_END
import pupil_apriltags as at
from collections import Counter

# ─── 参数 ───────────────────────────────────────────
ROBOT_NAME     = 'GomerX_Y6HFw4'
TAG_FAMILY     = 'tag36h11'
TAG_SIZE_M     = 0.02
CAMERA_PARAMS  = (650.0, 650.0, 400.0, 300.0)
TARGET_DIST_CM = 23.5
TOL_CM         = 0.5
MAX_ITER       = 50
MAX_STEP_CM    = 20
MIN_MARGIN     = 40
CHASSIS_STEP_MIN = 1.0


# ─── 内部状态 ───────────────────────────────────────
_detector = None
_camera   = None
_arm      = None
_chassis  = None
_window   = 'Detect Tag'


def _setup():
    global _detector
    _detector = at.Detector(
        families=TAG_FAMILY, nthreads=2, quad_decimate=2.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
    )


def scan(
    robot_name=ROBOT_NAME,
    scan_count=15,
    interval=0.4,
    show_window=True,
):
    """
    扫描视野中的 Tag，返回检测到最多的 Tag ID

    参数:
        robot_name:   机器人名称
        scan_count:   扫描帧数
        interval:     每帧间隔（秒）
        show_window:  是否显示实时窗口
    返回:
        int or None: 出现次数最多的 Tag ID，未检测到返回 None
    """
    print(f'[*] [扫描] 连接 {robot_name}...')
    my_robot = robot.Robot(robot_name)
    camera   = my_robot.camera
    camera.start_video_stream(display=False)
    time.sleep(1.0)

    _setup()
    window = 'Scan Tags'
    if show_window:
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    print(f'[*] [扫描] 开始扫描（{scan_count} 帧）...')
    id_counter = Counter()
    tag_data   = {}

    for i in range(scan_count):
        frame = camera.read_cv_image()
        if frame is None:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = _detector.detect(gray, estimate_tag_pose=True,
                               camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M)

        for tag in tags:
            color = (0, 255, 0)
            c = tag.corners.astype(int)
            cv2.polylines(frame, [c.reshape((-1, 1, 2))],
                          isClosed=True, color=color, thickness=2)
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv2.circle(frame, (cx, cy), 4, (0, 165, 255), -1)
            if tag.pose_t is not None:
                d = np.linalg.norm(tag.pose_t.flatten()) * 100
                cv2.putText(frame, f'ID={tag.tag_id} {d:.0f}cm',
                            (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            else:
                cv2.putText(frame, f'ID={tag.tag_id}',
                            (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            id_counter[tag.tag_id] += 1
            tag_data[tag.tag_id] = tag

        if show_window:
            cv2.imshow(window, frame)
            cv2.waitKey(1)
        time.sleep(interval)

    camera.stop_video_stream()
    my_robot.close()

    if show_window:
        cv2.destroyWindow(window)

    if id_counter:
        most_id, count = id_counter.most_common(1)[0]
        top3 = id_counter.most_common(3)
        print(f'\n[+] [扫描] 检测结果:')
        for tid, c in top3:
            tag = tag_data[tid]
            dist = ''
            if tag.pose_t is not None:
                dist = f'  距离≈{np.linalg.norm(tag.pose_t.flatten())*100:.0f}cm'
            print(f'    ID={tid}: {c}/{scan_count} 次{dist}')
        print(f'    → 返回 ID={most_id}')
        return most_id
    else:
        print('\n[!] [扫描] 未检测到任何 Tag')
        return None


def move_to_tag(
    robot_name=ROBOT_NAME,
    tag_id=0,
    target_dist_cm=TARGET_DIST_CM,
    tol_cm=TOL_CM,
    max_iter=MAX_ITER,
    max_step_cm=MAX_STEP_CM,
    show_window=True,
):
    """
    小车识别指定 AprilTag，移动到正前方 target_dist_cm 处停下。

    参数:
        robot_name:     机器人名称
        tag_id:         目标 Tag ID
        target_dist_cm: 目标停止距离（cm）
        tol_cm:         容忍误差（cm），也是底盘最小有效移动量
        max_iter:       最大迭代次数
        max_step_cm:    单次最大移动距离（cm）
        show_window:    是否显示实时检测窗口
    返回:
        bool: 是否成功到达目标
    """
    global _camera, _chassis, _arm

    print(f'[*] [对准] 连接 {robot_name}...')
    my_robot = robot.Robot(robot_name)
    _camera  = my_robot.camera
    _chassis = my_robot.chassis
    _arm     = my_robot.arm

    _camera.start_video_stream(display=False)
    time.sleep(1.5)

    _setup()

    if show_window:
        cv2.namedWindow(_window, cv2.WINDOW_AUTOSIZE)

    print(f'[*] [对准] 目标: Tag ID={tag_id} → {target_dist_cm}cm 前\n')

    for i in range(max_iter):
        frame = _camera.read_cv_image()
        if frame is None:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = _detector.detect(gray, estimate_tag_pose=True,
                               camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M)

        # 绘制
        for tag in tags:
            color = (0, 255, 0) if tag.tag_id == tag_id else (0, 128, 255)
            c = tag.corners.astype(int)
            cv2.polylines(frame, [c.reshape((-1, 1, 2))],
                          isClosed=True, color=color, thickness=2)
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv2.circle(frame, (cx, cy), 4, (0, 165, 255), -1)
            if tag.pose_t is not None:
                d = np.linalg.norm(tag.pose_t.flatten()) * 100
                cv2.putText(frame, f'ID={tag.tag_id} {d:.0f}cm',
                            (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            else:
                cv2.putText(frame, f'ID={tag.tag_id} (no pose)',
                            (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # ── 找目标 Tag（区分 pose / 无 pose）─────────────
        has_pose_t = None
        no_pose_t  = None
        for tag in tags:
            if tag.tag_id == tag_id:
                if tag.pose_t is not None and tag.decision_margin >= MIN_MARGIN:
                    has_pose_t = tag
                    break
                elif no_pose_t is None:
                    no_pose_t = tag

        if has_pose_t is not None:
            target = has_pose_t
            mode   = 'pose'
        elif no_pose_t is not None:
            target = no_pose_t
            mode   = 'center'
        else:
            # 无目标 Tag
            other = [t for t in tags if t.tag_id != tag_id]
            if other:
                cx_o = int(other[0].center[0])
                fw   = frame.shape[1]
                off  = (cx_o - fw / 2) / (fw / 2)
                print(f'[iter {i+1:02d}] ⚠️ 未找到 ID={tag_id}，有其他Tag旋转...')
                _arm.move_to(13, 19)
                time.sleep(0.2)
                _chassis.move(x=0, y=0, a=off * 30, wait_for_complete=True)
                time.sleep(0.3)
            else:
                print(f'[iter {i+1:02d}] ⚠️ 未找到 ID={tag_id}，后退重扫...')
                _arm.move_to(13, 19)
                time.sleep(0.2)
                _chassis.move(x=0, y=-5, a=0, wait_for_complete=True)
                time.sleep(0.3)
            if show_window:
                cv2.imshow(_window, frame)
                cv2.waitKey(1)
            continue

        # ── 计算移动量 ─────────────────────────────────
        if mode == 'pose':
            t = target.pose_t.flatten()
            tz_cm = float(t[2]) * 100
            tx_cm = float(t[0]) * 100
            err_z = tz_cm - target_dist_cm
            print(f'[iter {i+1:02d}] tz={tz_cm:.1f}cm  tx={tx_cm:+.1f}cm  '
                  f'err_z={err_z:+.1f}cm [pose]')
        else:
            fw = frame.shape[1]
            cx_t = int(target.center[0])
            tx_cm = (cx_t - fw / 2) / (fw / 2) * 40
            err_z = 20
            print(f'[iter {i+1:02d}] tz≈80cm(估)  tx≈{tx_cm:+.1f}cm(估)  [center]')

        if mode == 'pose' and abs(err_z) < tol_cm and abs(tx_cm) < tol_cm:
            print(f'\n✅ [对准] 到达！')
            _cleanup(my_robot, show_window)
            return True

        move_x = max(-max_step_cm, min(max_step_cm, tx_cm)) if abs(tx_cm) > tol_cm else 0.0
        move_y = max(-max_step_cm, min(max_step_cm, err_z)) if abs(err_z) > tol_cm else 0.0

        has_x = abs(move_x) >= CHASSIS_STEP_MIN
        has_y = abs(move_y) >= CHASSIS_STEP_MIN

        if not (has_x or has_y):
            print(f'\n✅ [对准] 进入最小步长范围，停止')
            _cleanup(my_robot, show_window)
            return True

        _arm.move_to(13, 19)
        time.sleep(0.2)
        _chassis.move(x=move_x if has_x else 0,
                     y=move_y if has_y else 0,
                     a=0, wait_for_complete=True)
        time.sleep(0.3)

        if show_window:
            cv2.imshow(_window, frame)
            cv2.waitKey(1)

    print(f'\n⚠️ [对准] 达到最大迭代 {max_iter}')
    _cleanup(my_robot, show_window)
    return False


def _cleanup(my_robot, show_window):
    """内部清理"""
    if show_window:
        cv2.destroyWindow(_window)
    _camera.stop_video_stream()
    my_robot.close()
    print('[*] 完成')
