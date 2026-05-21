"""
夹爪释放模块
"""
from gomerx import robot

ROBOT_NAME = 'GomerX_Y6HFw4'


def release(robot_name=ROBOT_NAME):
    """
    执行释放动作（高位→下降→开爪→避障位）

    参数:
        robot_name: 机器人名称
    """
    my_robot  = robot.Robot(robot_name)
    my_arm    = my_robot.arm
    my_gripper = my_robot.gripper

    my_arm.move_to(18, 19)        # 高位
    my_arm.move_to(18, 9)         # 垂直放下
    my_gripper.open()             # 释放
    my_arm.move_to(13, 19)        # 近点高位，不挡摄像头

    my_robot.close()
