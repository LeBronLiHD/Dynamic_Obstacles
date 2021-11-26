from vision import Vision
from vision import Point
from vision import Robot
import math
import parameters


def get_dis_delta(my_robot, target):
    distance = math.sqrt((my_robot.x - target.x) ^ 2 + (my_robot.y - target.y) ^ 2)
    tan_val = (target.y - my_robot.y) / (target.x - my_robot.x)
    delta = my_robot.orientation - math.atan(tan_val)
    return distance, delta


def if_we_could_full_speed(distance, current):
    acc_cur =
    zero_max = 0.5 * parameters.MAX_VEL_ACC * parameters.FRESH_CYCLE ^ 2
    all_max = parameters.FRESH_CYCLE * parameters.MAX_VEL
    return distance >= zero_max + all_max


def if_we_could_full_rudder(delta, current):
    zero_max = 0.5 * parameters.MAX_W_ACC * parameters.FRESH_CYCLE ^ 2
    all_max = parameters.MAX_W * parameters.FRESH_CYCLE
    return delta >= zero_max + all_max


def get_omage_vel(my_robot, target):
    distance, delta = get_dis_delta(my_robot, target)
    vel, w = 0, 0
    if if_we_could_full_rudder(delta):
        w = parameters.MAX_W

