#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
右移横扫 - 使用AprilTag检测
持续右移并检测指定Tag，找到目标ID后返回

用法:
  python sweep_right.py <target_ids> [exclude_ids]
  示例:
    python sweep_right.py 3,4         # 右移横扫，找id=3或4
    python sweep_right.py 3,4 1,2    # 右移横扫，找id=3或4，排除id=1,2
"""
import sys
import time
import cv2
import numpy as np
import pupil_apriltags as at
from gomerx import robot

ROBOT_NAME    = 'GomerX_Y6HFw4'
TAG_FAMILY    = 'tag36h11'
TAG_SIZE_M    = 0.02
CAMERA_PARAMS = (650.0, 650.0, 400.0, 300.0)
SWEEP_STEP    = 15        # 每次横移步长(cm)
SWEEP_TIMEOUT = 25      # 最大扫描时间(秒)


def sweep_right(target_ids=None, exclude_ids=None, robot_name=ROBOT_NAME):
    """
    右移横扫，持续右移并检测Tag

    参数:
        target_ids:  目标Tag ID列表（只识别这些ID）
        exclude_ids: 排除的Tag ID列表（忽略这些ID）
        robot_name:  机器人名称
    返回:
        int or None: 找到的Tag ID，未找到返回None
    """
    if target_ids is None:
        target_ids = []
    if exclude_ids is None:
        exclude_ids = []

    print(f'[*] [右横扫] 启动, 目标ID={target_ids}, 排除ID={exclude_ids}')

    my_robot = robot.Robot(robot_name)
    camera   = my_robot.camera
    chassis  = my_robot.chassis

    detector = at.Detector(
        families=TAG_FAMILY, nthreads=2, quad_decimate=2.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
    )

    camera.start_video_stream(display=False)
    time.sleep(1.0)

    window = 'Sweep Right'
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    start_time  = time.time()
    sweep_count = 0
    stream_stopped = False

    try:
        while time.time() - start_time < SWEEP_TIMEOUT:
            # 右移
            my_robot.arm.move_to(13, 19)
            time.sleep(0.1)
            chassis.move(x=SWEEP_STEP, y=0, a=0, wait_for_complete=True)
            time.sleep(0.2)
            sweep_count += 1

            # 检测
            frame = camera.read_cv_image()
            if frame is None or frame.size == 0:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray, estimate_tag_pose=True,
                                  camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M)

            # 绘制
            for tag in tags:
                if tag.tag_id in target_ids:
                    color = (0, 255, 0)
                elif tag.tag_id in exclude_ids:
                    color = (60, 60, 60)
                else:
                    color = (128, 128, 128)
                c = tag.corners.astype(int)
                cv2.polylines(frame, [c.reshape((-1, 1, 2))],
                             isClosed=True, color=color, thickness=2)
                cx, cy = int(tag.center[0]), int(tag.center[1])
                cv2.circle(frame, (cx, cy), 4, (0, 165, 255), -1)
                if tag.pose_t is not None:
                    d = np.linalg.norm(tag.pose_t.flatten()) * 100
                    cv2.putText(frame, f'ID={tag.tag_id} {d:.0f}cm',
                              (cx - 30, cy - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow(window, frame)
            cv2.waitKey(1)

            # 检查目标
            for tag in tags:
                is_target  = (tag.tag_id in target_ids) if target_ids else True
                is_exclude = tag.tag_id in exclude_ids
                if is_target and not is_exclude:
                    print(f'[+] [右横扫] 找到 ID={tag.tag_id} (横移{sweep_count}次)')
                    try:
                        camera.stop_video_stream()
                    except Exception:
                        pass
                    stream_stopped = True
                    try:
                        cv2.destroyWindow(window)
                    except Exception:
                        pass
                    my_robot.close()
                    return tag.tag_id

            if sweep_count % 10 == 0:
                print(f'[*] [右横扫] 已横移 {sweep_count} 次...')

        print('[!] [右横扫] 超时未找到目标')
        return None

    finally:
        if not stream_stopped:
            try:
                camera.stop_video_stream()
            except Exception:
                pass
        try:
            cv2.destroyWindow(window)
        except Exception:
            pass
        my_robot.close()


def main():
    # 解析命令行参数
    # 用法: sweep_right.py <target_ids> [exclude_ids]
    # 示例: sweep_right.py 3,4 1,2
    if len(sys.argv) < 2:
        print('用法: python sweep_right.py <target_ids> [exclude_ids]')
        print('示例: python sweep_right.py 3,4       # 找id=3或4')
        print('示例: python sweep_right.py 3,4 1,2  # 找id=3或4，排除id=1,2')
        sys.exit(1)

    target_ids = [int(x) for x in sys.argv[1].split(',')]

    exclude_ids = []
    if len(sys.argv) > 2:
        exclude_ids = [int(x) for x in sys.argv[2].split(',')]

    result = sweep_right(target_ids=target_ids, exclude_ids=exclude_ids)
    if result is not None:
        print(f'\n>>> 右横扫结果: ID={result}')
    else:
        print('\n>>> 右横扫结果: 未找到目标')


if __name__ == '__main__':
    main()
