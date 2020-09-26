"""
Simulate a quadrotor following a 3D trajectory

Author: Daniel Ingram (daniel-s-ingram)
"""

from math import cos, sin
import numpy as np
from Quadrotor import Quadrotor
from TrajectoryGenerator import TrajectoryGenerator
from mpl_toolkits.mplot3d import Axes3D
import math
from os import mkdir
from os.path import join, isdir
from my_utils import *
import matplotlib
import matplotlib.pyplot as plt

MAX_T = 100.0  # maximum time to the goal [s]
MIN_T = 1.0  # minimum time to the goal[s]
show_animation = True

class QuinticPolynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):

        # calc coefficient of quinic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[time**3, time**4, time**5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time**2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt



# Simulation parameters
g = 9.81
m = 0.2
Ixx = 1
Iyy = 1
Izz = 1
T = 60

# Proportional coefficients
Kp_x = 1
Kp_y = 1
Kp_z = 1
Kp_roll = 25
Kp_pitch = 25
Kp_yaw = 25

# Derivative coefficients
Kd_x = 10
Kd_y = 10
Kd_z = 1

# waypoints = [[-5, -5, 5], [5, -5, 5], [15, -5, 5], [25, -5, 5]]

waypoints = [[0, 10, 10], [500, 355, 10],[350, 10, 10],[500, 10, 10]]
num_waypoints = len(waypoints)


def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga,
                                max_vel, max_accel, max_jerk, dt):
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra_x, ra_y, ra, rj = [], [], [], [], [], [], [], [], []
    #acc_x_, acc_y_ = [], []
    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra_x, ra_y, ra, rj = [], [], [], [], [], [], [], [], []
        #acc_x_, acc_y_ = [], []

        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)
            ra_x.append(ax)
            ra_y.append(ay)

            '''acc_x_.append(ax)
            acc_y_.append(ay)'''

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in rv]) <= max_vel and max([abs(i) for i in ra]) <= max_accel and max(
                [abs(i) for i in rj]) <= max_jerk:
            print("find path!!")
            break

    return time, rx, ry, ryaw, rv, ra_x, ra_y, ra, rj



def quad_sim(x_c, y_c, z_c, i, time, rx, ry, ryaw, rv, ra_x, ra_y, ra, rj):
    """
    Calculates the necessary thrust and torques for the quadrotor to
    follow the trajectory described by the sets of coefficients
    x_c, y_c, and z_c.
    """
    x_pos = waypoints[0][0]
    y_pos = waypoints[0][1]
    z_pos = waypoints[0][2]


    x_acc = 0
    y_acc = 0
    z_acc = 0
    x_vel = 0
    y_vel = 0
    z_vel = 0
    roll = 0
    pitch = 0
    yaw = 0
    roll_vel = 0
    pitch_vel = 0
    yaw_vel = 0

    dist_goal = 0

    des_yaw = 0

    dt = 0.1
    t = 0

    q = Quadrotor(x=x_pos, y=y_pos, z=z_pos, roll=roll,
                  pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)

    i = 0
    n_run = num_waypoints - 1  # Number of waypoint-waypoint flights
    irun = 0
    o = 0


    while True:
        print("waypoints:", waypoints)

        start = waypoints[i]
        next_goal = waypoints[(i+1) % num_waypoints]

        goal_x = next_goal[0]
        goal_y = next_goal[1]
        goal_z = next_goal[2]


        dist_goal = distance_AB_2D(waypoints[i], waypoints[(i + 1) % num_waypoints])

        while dist_goal >= 1:
            print("Time:", t)
            PosizioneAttuale = np.array([x_pos, y_pos, z_pos])
            dist_goal = distance_AB_2D(PosizioneAttuale, next_goal)
            dist_percorsa2D = distance_AB_2D(start, PosizioneAttuale)

            # des_x_pos = calculate_position(x_c[i], t)
            # des_y_pos = calculate_position(y_c[i], t)
            des_z_pos = calculate_position(z_c[i], t)
            # des_x_vel = calculate_velocity(x_c[i], t)
            # des_y_vel = calculate_velocity(y_c[i], t)
            des_z_vel = calculate_velocity(z_c[i], t)

            des_x_acc = ra_x[o]
            print("o", o)
            print("RA_X-num", len(ra_x))
            print("des_x_acc", des_x_acc)
            des_y_acc = ra_y[o]
            print("des_y_acc", des_y_acc)
            des_z_acc = calculate_acceleration(z_c[i], t)

            thrust = m * (g + des_z_acc + Kp_z * (des_z_pos -
                                                  z_pos) + Kd_z * (des_z_vel - z_vel))

            roll_torque = Kp_roll * \
                (((des_x_acc * sin(des_yaw) - des_y_acc * cos(des_yaw)) / g) - roll)
            pitch_torque = Kp_pitch * \
                (((des_x_acc * cos(des_yaw) - des_y_acc * sin(des_yaw)) / g) - pitch)
            yaw_torque = Kp_yaw * (des_yaw - yaw)

            roll_vel += roll_torque * dt / Ixx
            pitch_vel += pitch_torque * dt / Iyy
            yaw_vel += yaw_torque * dt / Izz

            roll += roll_vel * dt
            pitch += pitch_vel * dt
            yaw += yaw_vel * dt

            R = rotation_matrix(roll, pitch, yaw)
            acc = (np.matmul(R, np.array(
                [0, 0, thrust.item()]).T) - np.array([0, 0, m * g]).T) / m
            x_acc = acc[0]
            y_acc = acc[1]
            z_acc = acc[2]
            x_vel += x_acc * dt
            y_vel += y_acc * dt
            z_vel += z_acc * dt
            x_pos += x_vel * dt
            y_pos += y_vel * dt
            z_pos += z_vel * dt

            q.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)

            acc_ms = math.sqrt(x_acc ** 2 + y_acc ** 2)
            vel_ms = math.sqrt(x_vel ** 2 + y_vel ** 2)

            # # # # # # #
            # Log info
            print("X_pos:","{:.2f}".format(x_pos) ,"\tX_vel (m/s):", "{:.2f}".format(x_vel), "\tX_acc (m/s^2):", "{:.2f}".format(x_acc))
            print("Y_pos:","{:.2f}".format(y_pos) ,"\tY_vel (m/s):", "{:.2f}".format(y_vel), "\tY_acc (m/s^2):", "{:.2f}".format(y_acc))
            print("Z_pos:","{:.2f}".format(z_pos) ,"\tZ_vel (m/s):", "{:.2f}".format(z_vel), "\tZ_acc (m/s^2):", "{:.2f}".format(z_acc))
            print("Acceleration:", "{:.2f}".format(acc_ms))
            print("Velocity (m/s):", "{:.2f}".format(vel_ms))
            print("dist_goal:", dist_goal)
            print("dist_percorsa2D", dist_percorsa2D)

            t += dt
            if (o == len(ra_x)-1):
                o = 0
            else:
                o = o + 1
        print("-" * 20, "[REACHED, Missing", distance_2D([x_pos, y_pos, z_pos], waypoints[(i + 1) % num_waypoints]),
              "m]", "-" * 20)

        t = 0
        o = 0
        i = (i + 1) % 4
        irun += 1

        if irun >= n_run:
            break
        print("-" * 20, "[PASSING TO NEXT WAYPOINT]", "-" * 20)



    print("Done")


def calculate_position(c, t):
    """
    Calculates a position given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial 
            trajectory generator.
        t: Time at which to calculate the position

    Returns
        Position
    """
    return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]


def calculate_velocity(c, t):
    """
    Calculates a velocity given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial 
            trajectory generator.
        t: Time at which to calculate the velocity

    Returns
        Velocity
    """
    return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]


def calculate_acceleration(c, t):
    """
    Calculates an acceleration given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial 
            trajectory generator.
        t: Time at which to calculate the acceleration

    Returns
        Acceleration
    """
    return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]


def rotation_matrix(roll, pitch, yaw):
    """
    Calculates the ZYX rotation matrix.

    Args
        Roll: Angular position about the x-axis in radians.
        Pitch: Angular position about the y-axis in radians.
        Yaw: Angular position about the z-axis in radians.

    Returns
        3x3 rotation matrix as NumPy array
    """
    return np.array(
        [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
         ])




def distance_2D(start, end):
    return math.sqrt((end[1] - start[1]) ** 2 + (end[0] - start[0]) ** 2)


def distance_3D(start, end):
    return math.sqrt((end[2] - start[2]) ** 2 + (end[1] - start[1]) ** 2 + (end[0] - start[0]) ** 2)

def main():
    """
    Print info header in the log
    """
    sys.stdout = Logger()
    print("\n\n\n" + "".join(["#"] * 50))
    if sys.platform.startswith('linux'):
        print("User:", format(getenv("USER")))  # For Linux
    if sys.platform.startswith('win32'):
        print("User:", format(getenv("USERNAME")))  # For Windows
    print("OS:", sys.platform)
    print("Date:", format(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]))
    print("".join(["#"] * 50) + "\n\n\n")

    """
    Calculates the x, y, z coefficients for the four segments 
    of the trajectory
    """
    x_coeffs = [[], [], [], []]
    y_coeffs = [[], [], [], []]
    z_coeffs = [[], [], [], []]

    sx = 0.0  # start x position [m]
    sy_l = 10.0  # start y position [m]
    sy_r = 0.0625
    syaw = np.deg2rad(0.0)  # start yaw angle [rad]
    sv_l = 0.0  # start speed [m/s]
    sa_l = 0.0  # start accel [m/ss]
    sv_r = 0.0  # start speed [m/s]
    sa_r = 0.0  # start accel [m/ss]

    gx = 500.0  # goal x position [m]
    gy = 355.0  # goal y position [m]
    gyaw = np.deg2rad(0.0)  # goal yaw angle [rad]
    gv = 0.0  # goal speed [m/s]
    ga = 0.0  # goal accel [m/ss]

    max_vel = 18  # max speed [m/s]
    max_accel = 3.0  # max accel [m/ss]
    max_jerk = 0.7  # max jerk [m/sss]
    dt = 0.1  # time tick [s]



    for i in range(num_waypoints):
        L = distance_2D(waypoints[i],waypoints[(i+1)%num_waypoints])
        '''if(i==0):
            vi = [0,0,0]
            vf = [13,0,0]
            ai = [0,0,0]
            af = [0,0,0]
        elif(i == 3):
            vi = [13,0,0]
            vf = [0,0,0]
            ai = [0,0,0]
            af = [0,0,0]
        else:
            vi = [13,0,0]
            vf = [13,0,0]
            ai = [0,0,0]
            af = [0,0,0]'''
        ok = (waypoints[(i + 1) % 4][0])
        ok1 = (waypoints[(i + 1) % 4][1])
        print("ok, ok1", ok, ok1)
        traj = TrajectoryGenerator(waypoints[i], waypoints[(i + 1) % 4], T)
        '''time_l, x_l, y_l, yaw_l, v_l, ra_x, ra_y, a_l, j_l = quintic_polynomials_planner(
            waypoints[i][0], waypoints[i][1], syaw, sv_l, sa_l, ok , ok1, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt)'''
        print("WAYPOINT", waypoints[i][0], waypoints[i][1], (waypoints[(i + 1) % 4][0]), (waypoints[(i + 1) % 4][1]))
        time_l, x_l, y_l, yaw_l, v_l, ra_x, ra_y, a_l, j_l = quintic_polynomials_planner(
            sx, sy_l, syaw, sv_l, sa_l, gx, gy, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt)

        traj.solve()
        x_coeffs[i] = traj.x_c
        y_coeffs[i] = traj.y_c
        z_coeffs[i] = traj.z_c

    quad_sim(x_coeffs, y_coeffs, z_coeffs, i , time_l, x_l, y_l, yaw_l, v_l, ra_x, ra_y, a_l, j_l)

if __name__ == "__main__":
    main()
