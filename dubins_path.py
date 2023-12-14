"""
MIT License

Copyright (c) 2020 Huiming Zhou

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

# class for PATH element
class PATH:
    def __init__(self, L, mode, x, y, yaw):
        self.L = L  # total path length [float]
        self.mode = mode  # type of each part of the path [string]
        self.x = x  # final x positions [m]
        self.y = y  # final y positions [m]
        self.yaw = yaw  # final yaw angles [rad]


# utils
def pi_2_pi(theta):
    while theta > math.pi:
        theta -= 2.0 * math.pi

    while theta < -math.pi:
        theta += 2.0 * math.pi

    return theta


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / math.pi / 2.0)


def LSL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_lsl = 2 + dist ** 2 - 2 * cos_a_b + 2 * dist * (sin_a - sin_b)

    if p_lsl < 0:
        return None, None, None, ["L", "S", "L"]
    else:
        p_lsl = math.sqrt(p_lsl)

    denominate = dist + sin_a - sin_b
    t_lsl = mod2pi(-alpha + math.atan2(cos_b - cos_a, denominate))
    q_lsl = mod2pi(beta - math.atan2(cos_b - cos_a, denominate))

    return t_lsl, p_lsl, q_lsl, ["L", "S", "L"]


def RSR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_rsr = 2 + dist ** 2 - 2 * cos_a_b + 2 * dist * (sin_b - sin_a)

    if p_rsr < 0:
        return None, None, None, ["R", "S", "R"]
    else:
        p_rsr = math.sqrt(p_rsr)

    denominate = dist - sin_a + sin_b
    t_rsr = mod2pi(alpha - math.atan2(cos_a - cos_b, denominate))
    q_rsr = mod2pi(-beta + math.atan2(cos_a - cos_b, denominate))

    return t_rsr, p_rsr, q_rsr, ["R", "S", "R"]


def LSR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_lsr = -2 + dist ** 2 + 2 * cos_a_b + 2 * dist * (sin_a + sin_b)

    if p_lsr < 0:
        return None, None, None, ["L", "S", "R"]
    else:
        p_lsr = math.sqrt(p_lsr)

    rec = math.atan2(-cos_a - cos_b, dist + sin_a + sin_b) - math.atan2(-2.0, p_lsr)
    t_lsr = mod2pi(-alpha + rec)
    q_lsr = mod2pi(-mod2pi(beta) + rec)

    return t_lsr, p_lsr, q_lsr, ["L", "S", "R"]


def RSL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_rsl = -2 + dist ** 2 + 2 * cos_a_b - 2 * dist * (sin_a + sin_b)

    if p_rsl < 0:
        return None, None, None, ["R", "S", "L"]
    else:
        p_rsl = math.sqrt(p_rsl)

    rec = math.atan2(cos_a + cos_b, dist - sin_a - sin_b) - math.atan2(2.0, p_rsl)
    t_rsl = mod2pi(alpha - rec)
    q_rsl = mod2pi(beta - rec)

    return t_rsl, p_rsl, q_rsl, ["R", "S", "L"]


def RLR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    rec = (6.0 - dist ** 2 + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0

    if abs(rec) > 1.0:
        return None, None, None, ["R", "L", "R"]

    p_rlr = mod2pi(2 * math.pi - math.acos(rec))
    t_rlr = mod2pi(
        alpha - math.atan2(cos_a - cos_b, dist - sin_a + sin_b) + mod2pi(p_rlr / 2.0)
    )
    q_rlr = mod2pi(alpha - beta - t_rlr + mod2pi(p_rlr))

    return t_rlr, p_rlr, q_rlr, ["R", "L", "R"]


def LRL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    rec = (6.0 - dist ** 2 + 2.0 * cos_a_b + 2.0 * dist * (sin_b - sin_a)) / 8.0

    if abs(rec) > 1.0:
        return None, None, None, ["L", "R", "L"]

    p_lrl = mod2pi(2 * math.pi - math.acos(rec))
    t_lrl = mod2pi(
        -alpha - math.atan2(cos_a - cos_b, dist + sin_a - sin_b) + p_lrl / 2.0
    )
    q_lrl = mod2pi(mod2pi(beta) - alpha - t_lrl + mod2pi(p_lrl))

    return t_lrl, p_lrl, q_lrl, ["L", "R", "L"]


def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):
    if m == "S":
        px[ind] = ox + l / maxc * math.cos(oyaw)
        py[ind] = oy + l / maxc * math.sin(oyaw)
        pyaw[ind] = oyaw
    else:
        ldx = math.sin(l) / maxc
        if m == "L":
            ldy = (1.0 - math.cos(l)) / maxc
        elif m == "R":
            ldy = (1.0 - math.cos(l)) / (-maxc)

        gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
        gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy

    if m == "L":
        pyaw[ind] = oyaw + l
    elif m == "R":
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions


def generate_local_course(L, lengths, mode, maxc, step_size):
    point_num = int(L / step_size) + len(lengths) + 3

    px = [0.0 for _ in range(point_num)]
    py = [0.0 for _ in range(point_num)]
    pyaw = [0.0 for _ in range(point_num)]
    directions = [0 for _ in range(point_num)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size

    ll = 0.0

    for m, l, i in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = -d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = interpolate(
                ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions
            )
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        px, py, pyaw, directions = interpolate(
            ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions
        )

    if len(px) <= 1:
        return [], [], [], []

    # remove unused data
    while len(px) >= 1 and px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions


def planning_from_origin(gx, gy, gyaw, curv, step_size):
    D = math.hypot(gx, gy)
    d = D * curv

    theta = mod2pi(math.atan2(gy, gx))
    alpha = mod2pi(-theta)
    beta = mod2pi(gyaw - theta)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    best_cost = float("inf")
    bt, bp, bq, best_mode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)

        if t is None:
            continue

        cost = abs(t) + abs(p) + abs(q)
        if best_cost > cost:
            bt, bp, bq, best_mode = t, p, q, mode
            best_cost = cost
    lengths = [bt, bp, bq]

    x_list, y_list, yaw_list, directions = generate_local_course(
        sum(lengths), lengths, best_mode, curv, step_size
    )

    return x_list, y_list, yaw_list, best_mode, best_cost


def calc_dubins_path(sx, sy, syaw, gx, gy, gyaw, curv, step_size=0.1):
    gx = gx - sx
    gy = gy - sy

    l_rot = Rot.from_euler("z", syaw).as_dcm()[0:2, 0:2]
    le_xy = np.stack([gx, gy]).T @ l_rot
    le_yaw = gyaw - syaw

    lp_x, lp_y, lp_yaw, mode, lengths = planning_from_origin(
        le_xy[0], le_xy[1], le_yaw, curv, step_size
    )

    rot = Rot.from_euler("z", -syaw).as_dcm()[0:2, 0:2]
    converted_xy = np.stack([lp_x, lp_y]).T @ rot
    x_list = converted_xy[:, 0] + sx
    y_list = converted_xy[:, 1] + sy
    yaw_list = [pi_2_pi(i_yaw + syaw) for i_yaw in lp_yaw]

    return PATH(lengths, mode, x_list, y_list, yaw_list)


def dubins_path(x, y, turning_radius, step_size=0.1):
    path = calc_dubins_path(
        x[0],
        x[1],
        math.radians(x[2]),
        y[0],
        y[1],
        math.radians(y[2]),
        turning_radius,
        step_size,
    )

    dubins_xx = []
    dubins_yy = []
    dubins_yaws = []

    for x, y, iyaw in zip(path.x, path.y, path.yaw):
        dubins_xx.append(x)
        dubins_yy.append(y)
        dubins_yaws.append(iyaw)

    return dubins_xx, dubins_yy, dubins_yaws

def has_loops(path, turning_radius, threshold_distance=0.1):
    # Get the coordinates of the path
    dubins_xx = []
    dubins_yy = []
    dubins_yaws = []

    for x, y, iyaw in zip(path.x, path.y, path.yaw):
        dubins_xx.append(x)
        dubins_yy.append(y)
        dubins_yaws.append(iyaw)

    # Ensure that all three arrays have the same length
    if len(dubins_xx) != len(dubins_yy) or len(dubins_xx) != len(dubins_yaws):
        raise ValueError("Arrays must have the same length.")
    
    loop_length = 2 * math.pi * turning_radius

    path_length = 0.0
    # find the path length
    for i in range(1, len(dubins_xx)):
        dx = dubins_xx[i] - dubins_xx[i - 1]
        dy = dubins_yy[i] - dubins_yy[i - 1]
        distance = math.sqrt(dx ** 2 + dy ** 2)
        path_length += distance

    # find straight line path length
    straight_line_distance = math.sqrt((dubins_xx[-1] - dubins_xx[0]) ** 2 + (dubins_yy[-1] - dubins_yy[0]) ** 2)

    margin = 0.5    # We consider 50% of a loop, the max amount by which a plan may deviate from a straight line.
    if (path_length - straight_line_distance) > margin * loop_length:
        return True
    
    return False
    

# Optimized dubins path: will find the best dubins path that won't create a loop
def optimized_dubins_path(x, y, step_size=0.1):
    initial_turning_radius = 5

    min_radius = 0.1
    max_radius = initial_turning_radius
    max_turning_radius = initial_turning_radius
    
    while (max_radius - min_radius) >= step_size:
        turning_radius = (max_radius + min_radius) / 2
        
        path = calc_dubins_path(
            x[0],
            x[1],
            math.radians(x[2]),
            y[0],
            y[1],
            math.radians(y[2]),
            1.0 / turning_radius,
            step_size,
        )
        
        if not has_loops(path, turning_radius):
            min_radius = turning_radius
            max_turning_radius = turning_radius
        else:
            max_radius = turning_radius

    print("max turning radius without loop: ", max_turning_radius)
    path = calc_dubins_path(
    x[0],
    x[1],
    math.radians(x[2]),
    y[0],
    y[1],
    math.radians(y[2]),
    1.0 / max_turning_radius,
    step_size,
)
        
    # Get the coordinates of the last optimal path
    dubins_xx = []
    dubins_yy = []
    dubins_yaws = []

    for x, y, iyaw in zip(path.x, path.y, path.yaw):
        dubins_xx.append(x)
        dubins_yy.append(y)
        dubins_yaws.append(iyaw)
    

    return dubins_xx, dubins_yy, dubins_yaws