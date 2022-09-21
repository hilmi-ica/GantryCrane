"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True

cpts = np.array([
    [[0,7.03], [23.5,7.03], [44,7.03], [64.5,7.03], [85,7.03], [105.5,7.03], [126,7.03], [150,7.03]],
    [[0,21.09], [23.5,21.09], [44,21.09], [64.5,21.09], [85,21.09], [105.5,21.09], [126,21.09], [150,21.09]],
    [[0,35.15], [23.5,35.15], [44,35.15], [64.5,35.15], [85,35.15], [105.5,35.15], [126,35.15], [150,35.15]],
    [[0,49.21], [23.5,49.21], [44,49.21], [64.5,49.21], [85,49.21], [105.5,49.21], [126,49.21], [150,49.21]],
    [[0,63.27], [23.5,63.27], [44,63.27], [64.5,63.27], [85,63.27], [105.5,63.27], [126,63.27], [150,63.27]],
])

def ob_box(xc):
    bottom = np.array([[x,xc[1]-7.03] for x in np.arange(xc[0]-6.6,xc[0]+6.6,3)])
    top = np.array([[x,xc[1]+7.03] for x in np.arange(xc[0]-6.6,xc[0]+6.6,3)])
    left = np.array([[xc[0]-6.6,y] for y in np.arange(xc[1]-7.03,xc[1]+7.03,3)])
    right = np.array([[xc[0]+6.6,y] for y in np.arange(xc[1]-7.03,xc[1]+7.03,3)])
    return np.concatenate((bottom,top,left,right))

def bound():
    line = np.array([[x,-5] for x in np.arange(-50,200.0,5)])
    return line

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


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 50  # [cm/s]
        self.min_speed = -5  # [cm/s]
        self.max_yaw_rate = 40 * math.pi / 180.0  # [rad/s]
        self.max_accel = 5 # [cm/ss]
        self.max_delta_yaw_rate = 40 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.1  # [cm/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1 # [s] Time tick for motion prediction
        self.predict_time = 1  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1
        self.obstacle_cost_gain = 0.5
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 10  # [cm] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 5  # [cm] for collision check
        self.robot_length = 12  # [cm] for collision check
        self.ob = np.array([])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")



def main(sx, sy, gx, gy, vacancy, robot_type=RobotType.rectangle):
    print(__file__ + " start!!")

    config.ob = obstacle(vacancy)

    # initial state [x(cm), y(cm), yaw(rad), v(cm/s), omega(rad/s)]
    # x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    x = np.array([sx*100, sy*100, math.pi / 2.0, 0.0, 0.0])
    # goal position [x(cm), y(cm)]
    goal = np.array([gx*100, gy*100])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            # plot_robot(x[0], x[1], x[2], config)
            # plot_arrow(x[0], x[1], x[2])
            plt.title("v[km/h]:" + str(round(predicted_trajectory[:,3][0]*0.036,5)))
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            # print("Goal!!")
            break

    print("Done")
    if show_animation:
        # plt.plot(goal[0], goal[1], "xb")
        # plt.plot(ob[:, 0], ob[:, 1], "ok")
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.title('Dynamic Window Optimal Crane Path')
        # plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

        # fig, ax = plt.subplots(2)
        # ax[0].plot(trajectory[:,3])
        # ax[0].grid(True)
        # ax[1].plot(trajectory[:,4])
        # ax[1].grid(True)
        # plt.pause(0.0001)

    plt.show()