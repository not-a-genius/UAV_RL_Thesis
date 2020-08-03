from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt
#sys.path.append(os.path.abspath("../custom_gym"))
from my_utils import *
from mpl_toolkits.mplot3d import Axes3D

from pylab import *
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
import os 

class Quadrotor():
    def __init__(self, id, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, size=0.25, battery_level=100, show_animation=True):
        self.id = id  # Unique (should be) identifier of drones
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p2 = np.array([-size / 2, 0, 0, 1]).T
        self.p3 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T

        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.show_animation = show_animation

        if self.show_animation:
            plt.ion()
            fig = plt.figure()
            # for stopping simulation with the esc key.
            fig.canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            self.ax = fig.add_subplot(111, projection='3d')

        self.update_pose(x, y, z, roll, pitch, yaw)

    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll    #Rollio fa spostare il drone verso destra o sinistra
        self.pitch = pitch  #Beccheggio fa avanzare il drone in avanti o indietro
        self.yaw = yaw      #Ruota il drone sul proprio asse o in senso orario o antiorario
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        if self.show_animation:
            self.plot()

    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = self.roll     #Spostamento verso destra o sinistra
        pitch = self.pitch   #Spostamento verso avanti o dietro
        yaw = self.yaw       #Rotazione sul proprio asse
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), z]
             ])

    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)


        # TODO show image at base of plot
        # fn = get_sample_data(os.getcwd()+"/lena.png", asfileobj=False)
        # img = read_png(fn)
        # x, y = ogrid[0:img.shape[0], 0:img.shape[1]]
        # self.ax.plot_surface(x, y,  np.atleast_2d(-2), rstride=5, cstride=5, facecolors=img)

        plt.cla()
        

        self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                     [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                     [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                     [p1_t[2], p2_t[2]], 'r-')
        self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                     [p3_t[2], p4_t[2]], 'r-')

        self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')

        plt.xlim(PLOTRANGE_X_NEG, PLOTRANGE_X_POS)
        plt.ylim(PLOTRANGE_Y_NEG, PLOTRANGE_Y_POS)
        self.ax.set_zlim(PLOTRANGE_Z_NEG, PLOTRANGE_Z_POS)

        plt.pause(0.001)
