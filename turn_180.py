"""
180度转向
用法: python turn_180.py
"""
from gomerx import robot

ROBOT_NAME = 'GomerX_Y6HFw4'


def turn_180():
    """180度掉头"""
    my_robot = robot.Robot(ROBOT_NAME)
    chassis = my_robot.chassis

    print('[*] 180度掉头...')
    chassis.move(x=0, y=0, a=180, wait_for_complete=True)
    print('[+] 完成')

    my_robot.close()


if __name__ == '__main__':
    turn_180()
