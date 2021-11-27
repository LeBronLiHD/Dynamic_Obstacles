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
    debugger = Debugger()
    left = Point(parameters.LEFT_X, parameters.LEFT_Y)
    right = Point(parameters.RIGHT_X, parameters.RIGHT_Y)
    if parameters.TEST_MODE == False:
        target = left
    else:
        target = Point(parameters.HEXAGON[0][0], parameters.HEXAGON[0][1])
    print("right point ->", right.x, right.y)
    print("left point  ->", left.x, left.y)
    debugger.draw_circle(left.x, left.y, 50)
    index = 0
    while True:
        if parameters.TEST_MODE == False:
            action.controlObs(vision)
        else:
            if calculation.in_circle(target, Point(vision.my_robot.x, vision.my_robot.y), 200):
                index += 1
                if index == 6:
                    print("one entire circle")
                    index = 0
                target = Point(parameters.HEXAGON[index][0], parameters.HEXAGON[index][1])
                debugger.draw_circle(target.x, target.y, 50)
                print("target changed to ->", target.x, target.y)
        tar_one, tar_two = calculation.get_target(vision.my_robot, vision, target=target, debugger=debugger)
        vx, vw = calculation.get_command(vision.my_robot, tar_one, tar_two)
        action.sendCommand(vx=vx, vy=0, vw=vw)
        time.sleep(parameters.FRESH_CYCLE)
        if parameters.TEST_MODE == False:
            target = calculation.switch_direction(left, right, vision.my_robot, target, debugger, radius=500)
