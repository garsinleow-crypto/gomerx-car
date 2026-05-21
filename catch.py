"""
夹爪抓取模块
"""
from gomerx import robot

ROBOT_NAME = 'GomerX_Y6HFw4'


def catch(robot_name=ROBOT_NAME):
    """
    执行抓取动作（开爪→进入→闭合→抬出→避障位）

    参数:
        robot_name: 机器人名称
    """
    my_robot  = robot.Robot(robot_name)
    my_arm    = my_robot.arm
    my_gripper = my_robot.gripper

    my_gripper.open()             # 抓夹保持打开
    my_arm.move_to(18, 9)         # 直接命中抓取位
    my_gripper.close()            # 抓取
    my_arm.move_to(18, 19)        # 垂直抽出
    my_arm.move_to(13, 19)        # 近点高位

    my_robot.close()
