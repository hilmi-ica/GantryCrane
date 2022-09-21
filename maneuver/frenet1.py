"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import copy
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PythonRobotics/PathPlanning/QuinticPolynomialsPlanner/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PythonRobotics/PathPlanning/CubicSpline/")

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise

SIM_LOOP = 500

# Parameter
# MAX_SPEED = 50.0 / 3.6  # maximum speed [cm/s]
# MAX_SPEED = 5  # maximum speed [cm/s]
MAX_SPEED = 50  # maximum speed [cm/s]
# MAX_ACCEL = 2.0  # maximum acceleration [cm/ss]
# MAX_ACCEL = 2  # maximum acceleration [cm/ss]
MAX_ACCEL = 20  # maximum acceleration [cm/ss]
# MAX_CURVATURE = 100  # maximum curvature [1/cm]
MAX_CURVATURE = 10  # maximum curvature [1/cm]
# MAX_ROAD_WIDTH = 7.0  # maximum road width [cm]
MAX_ROAD_WIDTH = 10.0  # maximum road width [cm]
# D_ROAD_W = 1.0  # road width sampling length [cm]
D_ROAD_W = 1  # road width sampling length [cm]
DT = 0.2  # time tick [s]
MAX_T = 5.0  # max prediction time [cm]
MIN_T = 4.0  # min prediction time [cm]
# TARGET_SPEED = 30.0 / 3.6  # target speed [cm/s]
# TARGET_SPEED = 0.3  # target speed [cm/s]
TARGET_SPEED = 40  # target speed [cm/s]
# D_T_S = 5.0 / 3.6  # target speed sampling length [cm/s]
# D_T_S = 0.02  # target speed sampling length [cm/s]
D_T_S = 5  # target speed sampling length [cm/s]
N_S_SAMPLE = 1  # sampling number of target speed
# ROBOT_RADIUS = 2.0  # robot radius [cm]
# ROBOT_RADIUS = 0.1  # robot radius [cm]
ROBOT_RADIUS = 1  # robot radius [cm]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = True

cpts = np.array([
    [[0,7.03], [23.5,7.03], [44,7.03], [64.5,7.03], [85,7.03], [105.5,7.03], [126,7.03], [150,7.03]],
    [[0,21.09], [23.5,21.09], [44,21.09], [64.5,21.09], [85,21.09], [105.5,21.09], [126,21.09], [150,21.09]],
    [[0,35.15], [23.5,35.15], [44,35.15], [64.5,35.15], [85,35.15], [105.5,35.15], [126,35.15], [150,35.15]],
    [[0,49.21], [23.5,49.21], [44,49.21], [64.5,49.21], [85,49.21], [105.5,49.21], [126,49.21], [150,49.21]],
    [[0,63.27], [23.5,63.27], [44,63.27], [64.5,63.27], [85,63.27], [105.5,63.27], [126,63.27], [150,63.27]],
])

class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []



def ob_box(xc):
    bottom = np.array([[x,xc[1]-7.03] for x in np.arange(xc[0]-6.6,xc[0]+6.6,3)])
    top = np.array([[x,xc[1]+7.03] for x in np.arange(xc[0]-6.6,xc[0]+6.6,3)])
    left = np.array([[xc[0]-6.6,y] for y in np.arange(xc[1]-7.03,xc[1]+7.03,3)])
    right = np.array([[xc[0]+6.6,y] for y in np.arange(xc[1]-7.03,xc[1]+7.03,3)])
    return np.concatenate((bottom,top,left,right))

def bound():
    line = np.array([[x,-5] for x in np.arange(-50,200.0,5)])
    return line

def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            # print(fplist[i].s_d)
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            # print(fplist[i].s_dd)
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            # print(fplist[i].c)
            continue
        elif not check_collision(fplist[i], ob):
            # print(check_collision(fplist[i],ob))
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)
    
    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def obstacle(vac):
    cpt = cpts[0:4,1:7,:]
    # print(cpt)
    idx = zip(*np.where(vac==1))
    # print(idx)
    obs = bound()
    for i in idx:
        # print(i)
        ob = ob_box(cpt[i])
        # print(cpt[i])
        obs = np.concatenate((obs,ob))
        # print(obs)
    return obs

def waypoint(sx,sy,gx,gy,vac):
    cpt = cpts[0:4,1:7,:]
    idx = zip(*np.where(vac==1))
    xi = [sx, gx]; yi = [sy, gy]
    wxs = []; wys = []
    wxg = []; wyg = []
    wxt = []; wyt = []
    for i in idx:
        x = cpt[i][0]
        y = cpt[i][1]
        if x < max(xi) and x > min(xi):
            if gx > sx:
                if x == sx+23.5:
                    wxs.append(sx); wys.append(y+7.03)
                elif x == sx+20.5:
                    wxs.append(sx); wys.append(y+7.03)
                if x == round(gx-20.5,5):
                    wxg.append(gx); wyg.append(y+7.03)
            elif gx < sx:
                if x == round(sx-20.5,5):
                    wxs.append(sx); wys.append(y+7.03)
                if x == gx+23.5:
                    wxg.append(gx); wyg.append(y+7.03)
                elif x == gx+20.5:
                    wxg.append(gx); wyg.append(y+7.03)
            wxt.append(x); wyt.append(y+14.06)
    if gx > sx:
        wx = [sx]; wy = [sy]
        if wxs:
            wx.append(wxs[wys.index(max(wys))])
            wy.append(wys[wys.index(max(wys))])
        if wxt:
            wx.append(wxt[wyt.index(max(wyt))])
            wy.append(wyt[wyt.index(max(wyt))])
        if wxg:
            wx.append(wxg[wyg.index(max(wyg))])
            wy.append(wyg[wyg.index(max(wyg))])
        wx.append(gx)
        wy.append(gy)
    elif gx < sx:
        wx = [gx]; wy = [gy]
        if wxg:
            wx.append(wxg[wyg.index(max(wyg))])
            wy.append(wyg[wyg.index(max(wyg))])
        if wxt:
            wx.append(wxt[wyt.index(max(wyt))])
            wy.append(wyt[wyt.index(max(wyt))])
        if wxs:
            wx.append(wxs[wys.index(max(wys))])
            wy.append(wys[wys.index(max(wys))])            
        wx.append(sx)
        wy.append(sy)
    return wx, wy

def main(sx, sy, gx, gy, vac):
    print(__file__ + " start!!")

    # wx, wy = waypoint(round(sx*10,5),round(sy*10,5),round(gx*10,5),round(gy*10,5),vac)
    wx, wy = waypoint(round(sx*100,5),round(sy*100,5),round(gx*100,5),round(gy*100,5),vac)  
    ob = obstacle(vac)

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
    
    # initial state
    # c_speed = 10.0 / 3.6  # current speed [cm/s]
    c_speed = 0  # current speed [cm/s]
    c_d = 0.0  # current lateral position [cm]
    c_d_d = 0.0  # current lateral speed [cm/s]
    c_d_dd = 0.0  # current lateral acceleration [cm/s]
    s0 = 0.0  # current course position

    # area = 10.0  # animation area length [cm]
    area = 100.0  # animation area length [cm]

    x_paths = []
    y_paths = []
    for i in range(SIM_LOOP):
        path = frenet_optimal_planning(
            csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        # if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 5:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            # plt.plot(tx, ty)
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            x_paths.append(path.x[1]); y_paths.append(path.y[1])
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 0.036)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:  # pragma: no cover
        # plt.plot(tx,ty)
        # plt.plot(ob[:, 0], ob[:, 1], "xk")
        plt.plot(x_paths,y_paths)
        plt.title('Frenet Optimal Crane Path')
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()