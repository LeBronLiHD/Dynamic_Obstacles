import algorithm
import parameters
from vision import Vision
from vision import Point
from vision import Robot
import math


def circle(debug, target):
    debug.draw_circle(target.x, target.y, 250)
    debug.draw_circle(target.x, target.y, 250 + 25)


def in_circle(center, point, radius=300):
    return math.sqrt((center.x - point.x) ** 2 + (center.y - point.y) ** 2) <= radius


def switch_direction(left, right, my_robot, target, debugger, radius=300):
    if in_circle(target, Point(my_robot.x, my_robot.y), radius=radius):
        if target == left:
            print("target switched to -> right ->", right.x, right.y)
            debugger.draw_circle(right.x, right.y, 50)
            return right
        else:
            print("target switched to -> left ->", left.x, left.y)
            debugger.draw_circle(left.x, left.y, 50)
            return left
    else:
        return target


def in_warning_area(vision, my_robot):
    in_area = [[], [], []]
    for i in range(16):
        if in_circle(Point(vision.obstacle(i).x, vision.obstacle(i).y),
                     Point(my_robot.x, my_robot.y),
                     radius=parameters.DETECT_RADIUS):
            in_area[0].append(i)
            distance, delta = algorithm.get_dis_delta(my_robot, vision.obstacle(i), is_obstacle=True)
            in_area[1].append(distance)
            in_area[2].append(delta)
    return in_area


def get_target(my_robot, global_vision, target, debugger):
    debugger.draw_circle(my_robot.x, my_robot.y, radius=parameters.DETECT_RADIUS)
    in_area = in_warning_area(global_vision, my_robot)
    # for i in range(len(in_area)):
    #     if parameters.DEBUG_IN_AREA:
    #         print(in_area[i])
    return target, target


def get_command(my_robot, tar_one, tar_two):
    vx, vw = algorithm.get_omage_vel(my_robot, tar_one)
    if parameters.DEBUG_IN_AREA:
        print("vx ->", vx, "   vw ->", vw)
    return vx, vw

