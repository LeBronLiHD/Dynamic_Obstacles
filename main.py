from vision import Vision
from vision import Robot
from vision import Point
from action import Action
from debug import Debugger
import calculation
import parameters
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debug = Debugger()
    right = Point(4500, 0)
    left = Point(-4500, 0)
    target = left
    print("right point ->", right.x, right.y)
    print("left point  ->", left.x, left.y)
    while True:
        action.controlObs(vision)
        tar_one, tar_two = calculation.get_target(vision.my_robot, vision, target=target)
        vx, vy, vw = calculation.get_command(vision.my_robot, tar_one, tar_two, target)
        action.sendCommand(vx=vx, vy=vy, vw=vw)
        time.sleep(parameters.FRESH_CYCLE)
        target = calculation.switch_direction(left, right, vision.my_robot, target, radius=250)
