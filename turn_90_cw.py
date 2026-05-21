"""
逆时针90度转向
用法: python turn_90_cw.py
"""
from gomerx import robot

ROBOT_NAME = 'GomerX_Y6HFw4'


def turn_90_cw():
    """逆时针转90度"""
    my_robot = robot.Robot(ROBOT_NAME)
    chassis = my_robot.chassis

    print('[*] 逆时针旋转90度...')
    chassis.move(x=0, y=0, a=-90, wait_for_complete=True)
    print('[+] 完成')

    my_robot.close()


if __name__ == '__main__':
    turn_90_cw()
