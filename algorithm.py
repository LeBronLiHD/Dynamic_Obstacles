from vision import Vision
from vision import Point
from vision import Robot
import math
import parameters


def get_dis_delta(my_robot, target, is_obstacle=False, debug=False):
    distance = math.sqrt((my_robot.x - target.x) ** 2 + (my_robot.y - target.y) ** 2)
    if target.x - my_robot.x != 0:
        tan_val = (target.y - my_robot.y) / (target.x - my_robot.x)
        sin_val = (target.y - my_robot.y) / distance
        orientation_target = math.atan(tan_val)
        if debug:
            print("tan ->", tan_val, "   sin ->", sin_val)
        if tan_val > 0 and sin_val < 0:
            orientation_target -= math.pi
        if tan_val < 0 and sin_val > 0:
            orientation_target += math.pi
    else:
        if target.y > my_robot.y:
            orientation_target = math.pi/2.0
        else:
            orientation_target = -math.pi/2.0
    if debug:
        print(my_robot.x, my_robot.y, my_robot.orientation, orientation_target)
    delta = my_robot.orientation - orientation_target
    if is_obstacle:
        delta = -delta
    if abs(delta) > math.pi:
        if delta < 0:
            delta += 2 * math.pi
        else:
            delta -= 2 * math.pi
        delta = -delta
    return distance, delta


def if_we_could_full_speed(distance, current):
    acc_time = (parameters.MAX_VEL - current)/parameters.MAX_VEL_ACC
    acc_cur = 0.5 * (parameters.MAX_VEL + current) * acc_time
    zero_max = (parameters.MAX_VEL - parameters.MAX_VEL_ACC * parameters.FRESH_CYCLE) * parameters.FRESH_CYCLE
    all_max = parameters.FRESH_CYCLE * parameters.MAX_VEL
    if parameters.DEBUG_IN_AREA:
        print("if_we_could_full_speed ->", distance, zero_max, all_max)
    return distance >= zero_max + all_max, acc_time >= parameters.FRESH_CYCLE


def if_we_could_full_rudder(delta, current):
    acc_time = (parameters.MAX_W - current) / parameters.MAX_W_ACC
    acc_cur = 0.5 * (parameters.MAX_W + current) * acc_time
    zero_max = (parameters.MAX_W - parameters.MAX_W_ACC * parameters.FRESH_CYCLE) * parameters.FRESH_CYCLE
    all_max = parameters.MAX_W * parameters.FRESH_CYCLE
    if parameters.DEBUG_IN_AREA:
        print("if_we_could_full_rudder ->", delta, zero_max, all_max)
    return abs(delta) >= zero_max + all_max, acc_time >= parameters.FRESH_CYCLE,


def if_over_limit(current):
    if abs(current) > parameters.MAX_W:
        if current > 0:
            return parameters.MAX_W
        else:
            return -parameters.MAX_W
    return current


def get_omage_vel(my_robot, target, is_simple=False):
    distance, delta = get_dis_delta(my_robot, target, debug=True)
    if parameters.DEBUG_IN_AREA:
        print("distance ->", distance, "   delta ->", delta)
    vel, w, flag_v, flag_w = 0, 0, False, False
    if is_simple:
        if abs(delta) > parameters.S_MAX_DELTA:
            abs_w = parameters.P_W * abs(delta)
            abs_w = min(abs_w, parameters.MAX_W)
            if parameters.S_CHANGE_W:
                parameters.S_CHANGE_W = False
                parameters.S_HAS_KEPT = 0
                if delta > 0:
                    parameters.S_W_DIRECTION = "left"
                    return 0, abs_w, distance, delta
                else:
                    parameters.S_W_DIRECTION = "right"
                    return 0, -abs_w, distance, delta
            else:
                parameters.S_HAS_KEPT += 1
                if parameters.S_HAS_KEPT >= parameters.S_KEEP_DIR:
                    parameters.S_CHANGE_W = True
                if parameters.S_W_DIRECTION == "left":
                    return 0, abs_w, distance, delta
                elif parameters.S_W_DIRECTION == "right":
                    return 0, -abs_w, distance, delta
                else:
                    print("Error! parameters.S_W_DIRECTION inValid!")
        else:
            parameters.S_CHANGE_W = True
            parameters.S_HAS_KEPT = 0
            p_vel = parameters.P_V * distance
            p_vel = min(parameters.MAX_VEL, p_vel)
            p_vel = max(parameters.S_MIN_VEL, p_vel)
            if distance > parameters.S_IN_TARGET_R:
                return p_vel, 0, distance, delta
            else:
                if parameters.DEBUG_IN_AREA * parameters.S_IN_RATIO:
                    print("already in target, return 0, 0")
                return 0, 0, distance, delta
    flag_w_1, flag_w_2 = if_we_could_full_rudder(delta, parameters.ROBOT_W)
    flag_v_1, flag_v_2 = if_we_could_full_speed(distance, my_robot.vel_x)
    if parameters.DEBUG_IN_AREA:
        print(flag_w_1, flag_w_2, flag_v_1, flag_v_2)
    if flag_w_1 and flag_w_2:
        if delta > 0:
            w = min(parameters.MAX_W, parameters.P_W * abs(delta))
        else:
            w = -min(parameters.MAX_W, parameters.P_W * abs(delta))
        flag_w = True
    if flag_v_1 and flag_v_2:
        vel = parameters.MAX_VEL
        flag_v = True
    if flag_w_1 and flag_w_2 == False:
        counter_clockwise = parameters.ROBOT_W + parameters.FRESH_CYCLE * parameters.MAX_W_ACC
        clockwise = parameters.ROBOT_W - parameters.FRESH_CYCLE * parameters.MAX_W_ACC
        counter_clockwise, clockwise = if_over_limit(counter_clockwise), if_over_limit(clockwise)
        if delta > 0:
            t1 = delta/counter_clockwise
            t2 = (2 * math.pi - delta)/clockwise
            if t1 >= t2:
                w = counter_clockwise
            else:
                w = clockwise
        else:
            t1 = abs(delta) / clockwise
            t2 = (2 * math.pi - abs(delta)) / counter_clockwise
            if t1 >= t2:
                w = clockwise
            else:
                w = counter_clockwise
        flag_w = True
    if flag_v_1 and flag_v_2 == False:
        vel = my_robot.vel_x + parameters.FRESH_CYCLE * parameters.MAX_VEL_ACC
        flag_v = True
    if flag_w and flag_v:
        parameters.ROBOT_W = w
        if flag_w and abs(w) > parameters.MAX_W/2.0:
            vel /= 4.0
        return vel, w, distance, delta
    elif flag_w:
        parameters.ROBOT_W = w
        return 500, w, distance, delta
    elif flag_v:
        parameters.ROBOT_W = 0
        return vel, 0, distance, delta
    else:
        parameters.ROBOT_W = 0
        print("impossible")
        return 500, 0, distance, delta

