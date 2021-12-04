import math

MAX_VEL = 3000
MAX_VEL_ACC = 3000
MAX_W = 5  # rad/s
MAX_W_ACC = 5  # rad/s^2

FRESH_CYCLE = 0.1

P_W = 2.5
I_W = 0
D_W = 0

P_V_W = 1

P_V = 0.75
I_V = 0
D_V = 0

DETECT_RADIUS = 375

LEFT_X = -3500
LEFT_Y = -2000
RIGHT_X = 3500
RIGHT_Y = 2000

DEBUG_IN_AREA = True
TEST_MODE = False
SIMPLE_MODE = True

S_W_DIRECTION = "left"
S_KEEP_DIR = 10
S_HAS_KEPT = 0
S_CHANGE_W = True
S_MAX_DELTA = math.pi/4
S_CONST_DELTA = 2.5

S_IN_RATIO = 0.75
S_MIN_VEL = 1000
S_CAKES = 12
S_TARGET_DIS = DETECT_RADIUS/2
S_TO_TARGET = DETECT_RADIUS * math.pi
S_W_DELTA = math.pi

S_IN_TARGET_R = S_TARGET_DIS * 0.5

SET_ROUNDS = 3000

ROBOT_W = 0  # last w of my_robot

HEXAGON = [[-1750, 2000],
           [-3500, 0],
           [-1750, -2000],
           [1750, 2000],
           [3500, 0],
           [1750, -2000]]
