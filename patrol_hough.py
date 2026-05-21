"""
霍夫变换巡线 - 最精简版
使用OpenCV霍夫变换检测线段，SDK巡线
超过3秒未检测到线则退出
"""
import cv2
import numpy as np
import argparse
import time
from gomerx import robot
from gomerx.skill import LINE_END

ROBOT_NAME = 'GomerX_Y6HFw4'

# ============ SDK HSV参数 (2025-05-19 标定) ============
# 黄色: SDK格式 (H:0-360, S:0-100, V:0-100)
YELLOW_LOW  = (0, 23, 60)    # OpenCV(0,59,153) -> SDK
YELLOW_HIGH = (92, 100, 100)  # OpenCV(46,255,255) -> SDK

# 蓝色: SDK格式
BLUE_LOW  = (60, 10, 20)      # OpenCV(30,25,50) -> SDK
BLUE_HIGH = (300, 78, 95)     # OpenCV(150,199,242) -> SDK

# ============ 巡线参数 ============
ROI_TOP = 0.50           # 只检测画面下半1/2
HOUGH_THRESH = 25        # 霍夫阈值
HOUGH_MIN_LEN = 25       # 最小线长


def detect_line_hough(frame, color='yellow'):
    """霍夫变换检测线段"""
    h, w = frame.shape[:2]
    roi = frame[int(h*ROI_TOP):h, 0:w]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    if color == 'yellow':
        # OpenCV格式 (H:0-180, S:0-255, V:0-255) - 2025-05-19标定
        lower = np.array([0, 59, 153])
        upper = np.array([46, 255, 255])
    else:
        # 蓝色 - OpenCV格式 - 2025-05-19标定
        lower = np.array([30, 25, 50])
        upper = np.array([150, 199, 242])

    mask = cv2.inRange(hsv, lower, upper)

    # 形态学处理
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 边缘检测
    edges = cv2.Canny(mask, 50, 150)

    # 霍夫变换
    lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                            threshold=HOUGH_THRESH,
                            minLineLength=HOUGH_MIN_LEN,
                            maxLineGap=15)

    if lines is None or len(lines) == 0:
        return None, w, int(h*ROI_TOP)

    # 取最长的线
    longest = max(lines, key=lambda l: np.sqrt((l[0][2]-l[0][0])**2 + (l[0][3]-l[0][1])**2))
    return longest, w, int(h*ROI_TOP)


def draw_debug(frame, line, found):
    """绘制调试信息"""
    if line is not None:
        x1, y1, x2, y2 = line[0]
        roi = int(frame.shape[0] * ROI_TOP)
        cv2.line(frame, (x1, y1+roi), (x2, y2+roi), (0, 255, 255), 3)
    status = "Found" if found else "Lost"
    cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Debug', frame)
    cv2.imshow('Mask', cv2.resize(frame[int(frame.shape[0]*ROI_TOP):], (400, 200)))


def patrol_hough(color='yellow', debug=True, robot_name=None, once=False):
    """霍夫变换巡线 - 检测到线后执行巡线；超过3秒未检测到线则退出

    Args:
        color: 'yellow' 或 'blue'
        debug: 是否显示调试窗口
        robot_name: 机器人名称
        once: True则只检测一次后返回，False则无限循环（但有3秒超时）
    """
    if robot_name is None:
        robot_name = ROBOT_NAME

    print(f"[*] 连接机器人: {robot_name}")
    my_robot = robot.Robot(robot_name)
    camera = my_robot.camera
    skill = my_robot.skill

    camera.start_video_stream(display=False)

    if color == 'yellow':
        hsv_low, hsv_high = YELLOW_LOW, YELLOW_HIGH
    else:
        hsv_low, hsv_high = BLUE_LOW, BLUE_HIGH

    print(f"[*] 巡{color}线 - HSV: {hsv_low} ~ {hsv_high}")

    if debug:
        cv2.namedWindow('Debug', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)

    start_time = time.time()
    found_line = False
    try:
        while True:
            # 3秒超时检测
            if time.time() - start_time > 3:
                print("[!] 3秒内未检测到线，退出")
                break

            frame = camera.read_cv_image()
            if frame is None:
                continue

            # 霍夫检测
            line, _, _ = detect_line_hough(frame, color)

            # 调试显示
            if debug:
                draw_debug(frame, line, line is not None)
                if cv2.waitKey(1) == 27:
                    break

            if line is not None:
                found_line = True
                print("[+] 检测到线段，执行巡线...")
                camera.stop_video_stream()
                skill.detect_line(hsv_low=hsv_low, hsv_high=hsv_high, timeout=5)
                result = skill.move_along_line(stop=LINE_END)
                camera.start_video_stream(display=False)
                if result:
                    print("[+] 巡线完成")
                else:
                    print("[!] 巡线中断")
                break

    except KeyboardInterrupt:
        print("\n[*] 中断")
    finally:
        camera.stop_video_stream()
        my_robot.close()
        if debug:
            cv2.destroyAllWindows()
        print("[*] 结束")


def main():
    parser = argparse.ArgumentParser(description='霍夫变换巡线')
    parser.add_argument('--robot', '-r', default=ROBOT_NAME, help='机器人名称')
    parser.add_argument('--color', '-c', choices=['yellow', 'blue'], default='yellow')
    parser.add_argument('--debug', '-d', action='store_true', help='调试模式')
    args = parser.parse_args()

    patrol_hough(color=args.color, debug=args.debug, robot_name=args.robot)


if __name__ == '__main__':
    main()
