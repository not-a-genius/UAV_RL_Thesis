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
from QuinticPolynomial import QuinticPolynomial
import pandas as pd

MAX_T = 500.0   # maximum time to the goal[s]
MIN_T = 1.0     # minimum time to the goal[s]
show_animation = True

#----------------------------------------------------------------------------------------------------------------------------------#
info = []

info1 = "\n\n___________________________________________ENVIRONMENT AND DRONE INFO: ___________________________________________\n"
info.append(info1)
info24 = "\nWP3 SCENARIO: " + str(scenario)
info.append(info24)
info2 = "\nDRONE ID: " + str(id)
info.append(info2)
info3 = "\nVOL: " + str(vol)
info.append(info3)
info4 = "\nAIR RISK: " + str(AirRisk)
info.append(info4)
info5 = "\nGROUND RISK: " + str(GroundRisk)
info.append(info5)
info6 = "\nOP TYPE: " + str(Op_Type)
info.append(info6)
info7 = "\nTYPE OF DRONE: " + str(model)
info.append(info7)
info8 = "\nDIMENSION: " + str(Dimension) + " m"
info.append(info8)
info9 = "\nMASS: " + str(m) + " Kg"
info.append(info9)
info22 = "\nCRUISE SPEED: " + str(cruise_speed_ms) + " m/s"
info.append(info22)
info10 = "\nVDR: " + str(VRD) + " m/s"
info.append(info10)
info11 = "\nVRC: " + str(VRC) + " m/s"
info.append(info11)
info12 = "\nStationary Max: " + str(stationary) + " min"
info.append(info12)
info13 = "\nMAX WIND: " + str(Maxwind) + " m/s"
info.append(info13)
info14 = "\nPAYLOAD RISK: " + str(PayloadRisk)
info.append(info14)
info15 = "\nT. TYPE: " + str(T_Type)
info.append(info15)
info16 = "\nFLIGHT MODE: " + str(FlightMode)
info.append(info16)
info17 = "\nMONITORING: " + str(Monitoring)
info.append(info17)
info18 = "\nTRACKING SERVICE: " + str(TrackingService)
info.append(info18)
info19 = "\nTACTICAL SEPARATION: " + str(TacticalSeparation)
info.append(info19)
info20 = "\nDISTANCE TO REACH THE GOAL: " + str(distance_space_m) + " m"
info.append(info20)
info21 = "\nMINIMUM AND MAXIMUM TIME TO REACH THE GOAL: " + str(MIN_T) + " s - " + str(MAX_T) + " s"
info.append(info21)
info25 = "\nSTART COORDINATES: " + "X:" + str(start_pos[0]) + " Y:" + str(start_pos[1]) + " Z:" + str(start_pos[2])
info.append(info25)
info26 = "\nDESTINATION COORDINATES(1): " + "X:" + str(waypoint_dest[0]) + " Y:" + str(waypoint_dest[1]) + " Z:" + str(waypoint_dest[2])
info.append(info26)
if(add_waypoint):
    info27 = "\nDESTINATION COORDINATES(2): " + "X:" + str(add_waypoint[0]) + " Y:" + str(add_waypoint[1]) + " Z:" + str(add_waypoint[2])
    info.append(info27)
info28 = "\nAltitude: " + str(altitude) + " m"
info.append(info28)

info23 = "\n__________________________________________________________________________________________________________________\n\n"
info.append(info23)

if not isdir(LOG_DIRECTORY_NAME): mkdir(LOG_DIRECTORY_NAME)
file = open(LOG_PATH, "w")

for i in info:
    print(i)
    file.write(i)
file.close()
#----------------------------------------------------------------------------------------------------------------------------------#

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


waypoints = [[0, 10, 10], [500, 355, 10],[-500, 355, 10],[0, 10, 10]]
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

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra_x, ra_y, ra, rj = [], [], [], [], [], [], [], [], []


        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            '''rx_arr = np.array(rx)
            ry_arr = np.array(ry)
            #print("rx no noise:", rx_arr)
            #print("ry no noise:", ry_arr)
            noise_x = np.random.normal(0, 0.1, rx_arr.shape)
            noise_y = np.random.normal(0, 0.1, ry_arr.shape)
            rx_noise_ = rx_arr + noise_x
            ry_noise_ = ry_arr + noise_y
            rx_noise = rx_noise_.tolist()
            ry_noise = ry_noise_.tolist()
            rx = rx_noise
            ry = ry_noise
            #print("rx yes noise:", rx)
            #print("ry yes noise:", ry)'''

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


            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in rv]) <= max_vel and max([abs(i) for i in ra]) <= max_accel and max(
                [abs(i) for i in rj]) <= max_jerk:
            print("find path!!")
            '''print("rx", rx)
            print("ry:", ry)
            print("rx_noise:", rx_noise)
            print("ry_noise:", ry_noise)'''
            break

    return time, rx, ry, ryaw, rv, ra_x, ra_y, ra, rj



def quad_sim(x_c, y_c, z_c, i, time, rx, ry, yaw_l_tot, rv, ra_x, ra_y, ra, rj, ra_x_tot, ra_y_tot):
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

    x_pos_tot = []
    y_pos_tot = []
    x_vel_tot = []
    y_vel_tot = []
    x_acc_tot = []
    y_acc_tot = []
    time_tot  = []
    parameters = []

    dt = 0.1
    t = 0

    q = Quadrotor(x=x_pos, y=y_pos, z=z_pos, roll=roll,
                  pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)

    i = 0
    n_run = num_waypoints - 1  # Number of waypoint-waypoint flights
    irun = 0
    o = 0

    #print("ra_x_tot", ra_x_tot)
    while True:
        #print("waypoints:", waypoints)
        #print("No-noise ra_x_tot[i]-waypoints", ra_x_tot[i])
        start = waypoints[i]
        next_goal = waypoints[(i+1) % num_waypoints]


        goal_x = next_goal[0]
        goal_y = next_goal[1]
        goal_z = next_goal[2]


        dist_goal = distance_AB_2D(waypoints[i], waypoints[(i + 1) % num_waypoints])
        n_ra_x_tot = len(ra_x_tot[i])
        #Gaussian-noise--------------------------------------------------------------#
        ra_x_tot_arr = np.array(ra_x_tot[i])
        ra_y_tot_arr = np.array(ra_y_tot[i])
        noise_x = np.random.normal(0, .05, ra_x_tot_arr.shape)
        noise_y = np.random.normal(0, .05, ra_x_tot_arr.shape)
        ra_x_noise = ra_x_tot_arr + noise_x
        ra_y_noise = ra_y_tot_arr + noise_y
        # Gaussian-noise--------------------------------------------------------------#
        #print("noise:", noise_x)
        #print("new_signal:", ra_x_noise)
        #print("ra_x_tot[i]", ra_x_tot)
        #print("ra_x_noise", ra_x_noise)


        print("n_ra_x_tot", n_ra_x_tot)



        while n_ra_x_tot != o:
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
            #des_x_acc = ra_y_tot[i][o]
            des_x_acc = ra_x_noise[o]
            print("Iterazione_o:", o)
            print("RA_X-num", len(ra_x_tot[i]))
            print("des_x_acc", des_x_acc)
            #des_y_acc = ra_y_tot[i][o]
            des_y_acc = ra_y_noise[o]
            print("RA_y-num", len(ra_y_tot[i]))
            print("des_y_acc", des_y_acc)

            des_z_acc = calculate_acceleration(z_c[i], t)

            #des_yaw = yaw_l_tot[i][o]
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
            z_acc = 0
            x_vel += x_acc * dt
            y_vel += y_acc * dt
            z_vel += z_acc * dt
            x_pos += x_vel * dt
            y_pos += y_vel * dt
            z_pos += z_vel * dt

            x_pos_tot.append(x_pos)
            y_pos_tot.append(y_pos)
            x_vel_tot.append(x_vel)
            y_vel_tot.append(y_vel)
            x_acc_tot.append(x_acc)
            y_acc_tot.append(y_acc)
            time_tot.append(t)





            q.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)

            acc_ms = np.hypot(x_acc, y_acc)
            vel_ms = np.hypot(x_vel, y_vel)

            # # # # # # #
            # Log info
            print("X_pos:","{:.2f}".format(x_pos) ,"\tX_vel (m/s):", "{:.2f}".format(x_vel), "\tX_acc (m/s^2):", "{:.2f}".format(x_acc))
            print("Y_pos:","{:.2f}".format(y_pos) ,"\tY_vel (m/s):", "{:.2f}".format(y_vel), "\tY_acc (m/s^2):", "{:.2f}".format(y_acc))
            print("Z_pos:","{:.2f}".format(z_pos) ,"\tZ_vel (m/s):", "{:.2f}".format(z_vel), "\tZ_acc (m/s^2):", "{:.2f}".format(z_acc))
            print("Roll:", "{:.4f}".format(roll), "\tPitch:", "{:.4f}".format(pitch), "\tYaw", "{:.4f}".format(yaw))
            print("Acceleration:", "{:.2f}".format(acc_ms))
            print("Velocity (m/s):", "{:.2f}".format(vel_ms))
            print("dist_goal:", dist_goal)
            print("dist_percorsa2D", dist_percorsa2D)

            '''for a,b,c,d,e,f in zip(*parameters):
                print(a,b,c,d,e,f)'''

            t += dt
            o = o + 1

        print("-" * 20, "[REACHED, Missing", distance_2D([x_pos, y_pos, z_pos], waypoints[(i + 1) % num_waypoints]),
              "m]", "-" * 20)
        parameters.append(x_acc_tot)
        parameters.append(y_acc_tot)
        parameters.append(x_vel_tot)
        parameters.append(y_vel_tot)
        parameters.append(x_pos_tot)
        parameters.append(y_pos_tot)
        parameters.append(time_tot)
        print("Time|x_acc|y_acc|x_vel|y_vel|x_pos|y_pos")
        for a,b,c,d,e,f,l in zip(*parameters):
            print("{:.2f}".format(l),"{:.3f}".format(a),"{:.3f}".format(b),"{:.3f}".format(c),
                  "{:.3f}".format(d),"{:.3f}".format(e),"{:.3f}".format(f))


        t = 0
        o = 0
        i = (i + 1) % 4
        irun += 1

        if irun >= n_run:
            break
        print("-" * 20, "[PASSING TO NEXT WAYPOINT]", "-" * 20)
        print("x_pos_tot", x_pos_tot)
        #print("y_pos_tot", y_pos_tot)



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

    #sx = 0.0  # start x position [m]
    #sy_l = 10.0  # start y position [m]
    sy_r = 0.0625
    syaw = np.deg2rad(5.0)  # start yaw angle [rad]
    sv_l = 0.0  # start speed [m/s]
    sa_l = 0.0  # start accel [m/ss]
    sv_r = 0.0  # start speed [m/s]
    sa_r = 0.0  # start accel [m/ss]

    #gx = 500.0  # goal x position [m]
    #gy = 355.0  # goal y position [m]
    gyaw = np.deg2rad(10.0)  # goal yaw angle [rad]
    gv = 0.0  # goal speed [m/s]
    ga = 0.0  # goal accel [m/ss]

    max_vel = 18  # max speed [m/s]
    max_accel = 3.0  # max accel [m/ss]
    max_jerk = 3  # max jerk [m/sss]
    dt = 0.1  # time tick [s]

    ra_x_tot = [] #Array di tutte le ra_x (accelerazione sull'asse x) per ogni waypoints
    ra_y_tot = [] #Array di tutte le ra_y (accelerazione sull'asse y) per ogni waypoints
    yaw_l_tot = []

    for i in range(num_waypoints):
        #L = distance_2D(waypoints[i],waypoints[(i+1)%num_waypoints])
        next_waypoints_x = (waypoints[(i + 1) % 4][0])
        next_waypoints_y = (waypoints[(i + 1) % 4][1])
        traj = TrajectoryGenerator(waypoints[i], waypoints[(i + 1) % 4], T)

        time_l, x_l, y_l, yaw_l, v_l, ra_x, ra_y, a_l, j_l = quintic_polynomials_planner(
            waypoints[i][0], waypoints[i][1], syaw, sv_l, sa_l, next_waypoints_x , next_waypoints_y, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt)

        print("WAYPOINT", waypoints[i][0], waypoints[i][1], "|" , (waypoints[(i + 1) % 4][0]), (waypoints[(i + 1) % 4][1]))
        '''time_l, x_l, y_l, yaw_l, v_l, ra_x, ra_y, a_l, j_l = quintic_polynomials_planner(
            sx, sy_l, syaw, sv_l, sa_l, gx, gy, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt)'''

        ra_x_tot.append(ra_x) #Array di tutte le ra_x (accelerazione sull'asse x) per ogni waypoints
        ra_y_tot.append(ra_y) #Array di tutte le ra_y (accelerazione sull'asse y) per ogni waypoints

        yaw_l_tot.append(yaw_l)  # Array di tutte le yaw_l (yaw sull'asse x) per ogni waypoints


        traj.solve()
        x_coeffs[i] = traj.x_c
        y_coeffs[i] = traj.y_c
        z_coeffs[i] = traj.z_c


    quad_sim(x_coeffs, y_coeffs, z_coeffs, i , time_l, x_l, y_l, yaw_l_tot, v_l, ra_x, ra_y, a_l, j_l, ra_x_tot, ra_y_tot)

if __name__ == "__main__":
    main()
