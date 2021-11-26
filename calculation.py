from vision import Vision
from vision import Point
from vision import Robot
import math


def circle(debug, target):
    debug.draw_circle(target.x, target.y, 250)
    debug.draw_circle(target.x, target.y, 250 + 25)


def in_circle(center, point, radius=300):
    return math.sqrt((center.x - point.x) ^ 2 + (center.y - point.y) ^ 2) <= radius


def switch_direction(left, right, my_robot, target, radius=300):
    if in_circle(target, Point(my_robot.x, my_robot.y), radius=radius):
        if target == left:
            print("target switched to -> right")
            return right
        else:
            print("target switched to -> left")
            return left
    else:
        return target


def get_target(my_robot, global_vision, target):
    return target, target


def get_command(my_robot, tar_one, tar_two, target):
    if target.x < 0:
        return 1000, 0, 0
    else:
        return -1000, 0, 0

