import algorithm
import parameters
from vision import Vision
from vision import Point
from vision import Robot
import math
import heapq
import numpy as np


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


def transfer(angle, abs_value=False):
    if (angle <= math.pi):
        return angle
    else:
        if abs_value == False:
            return angle - 2 * math.pi
        else:
            return 2 * math.pi - angle


def get_target(my_robot, global_vision, target, debugger):
    debugger.draw_circle(my_robot.x, my_robot.y, radius=parameters.DETECT_RADIUS)
    in_area = in_warning_area(global_vision, my_robot)
    dis,delta = algorithm.get_dis_delta(my_robot, target, is_obstacle=False, debug=False)
    for i in range(len(in_area)):
        if parameters.DEBUG_IN_AREA:
            print(in_area[i])

    rounds = [0 for i in range(0, parameters.S_CAKES)]

    print(len(in_area[0]))
    max_line = len(in_area[0])
    for i in range(parameters.S_CAKES): #分为——12*30°
        for j in range(max_line):
            rounds[i] += (parameters.DETECT_RADIUS - in_area[1][j]) * \
                         abs(math.pi - transfer(abs(transfer(math.pi/12*i) - in_area[2][j]), abs_value=True))
        print( rounds[i])
    best_orein = 0
    min_index = []
    min_orein = heapq.nsmallest(3, rounds)
    angle_rounds = [0 for i in range(3)]
    for t in min_orein:
        index = rounds.index(t)
        min_index.append(index)
        rounds[index] = 0
    for i in range (0,3):
        for j in range(max_line):
            angle_rounds[i] += abs(math.pi - transfer(abs(transfer(math.pi / 12 * min_index[i]) - in_area[2][j])))
    print(min_index)
    print(min_orein)
    for i in range(3):
        if (i == 0):
            best_angle= angle_rounds[0]
            best_orein = 0
        if (angle_rounds[i] < best_orein):
            best_angle = angle_rounds[i]
            best_orein = i
    best_angle = min_index[best_orein] * math.pi/12
    print(best_angle)
    target = Point(my_robot.x + parameters.S_TARGET_DIS * math.cos(best_angle * math.pi/12 + my_robot.orientation),
                   my_robot.y + parameters.S_TARGET_DIS * math.sin(best_angle * math.pi/12 + my_robot.orientation))
    debugger.draw_circle(target.x,target.y,radius=50)
    return target, target


def get_command(my_robot, tar_one, tar_two):
    vx, vw, dis, delta = algorithm.get_omage_vel(my_robot, tar_one, is_simple=parameters.SIMPLE_MODE)
    if parameters.SIMPLE_MODE == False:
        if abs(vw) <= abs(delta * parameters.P_W) or vw * delta < 0:
            if abs(delta * parameters.P_W) < parameters.MAX_W:
                vw = delta * parameters.P_W
    if parameters.DEBUG_IN_AREA:
        print("vx ->", vx, "   vw ->", vw)
    return vx, vw

