import matplotlib.pyplot as plt
import numpy as np

class PIDPlotter():
    # def __init__(self,pause_dt):
    #     self.xs = []
    #     self.ys_error = []
    #     self.ys_ref = []
    #     self.pause = pause_dt

    def __init__(self,pause_dt):
        self.pause = pause_dt
        self.hl, = plt.plot([],[])
        self.hl2, = plt.plot([],[],c="r")
        self.ax = plt.axes()

    # def animate(self,i):
    #     self.xs.append(nx)
    #     self.ys_error.append(ny_err)
    #     self.ys_ref.append(ny_ref)

    #     plt.cla()
    #     plt.plot(self.xs,self.ys_error[i], label="ERROR")
    #     plt.plot(self.xs,self.ys_ref[i], label="REF")
        
    #     plt.legend(loc = "upper left")
    #     plt.tight_layout()


    def drawLine(self,current_hl,x,y):
        current_hl.set_xdata(np.append(current_hl.get_xdata(),x))
        current_hl.set_ydata(np.append(current_hl.get_ydata(),y))
        

    def animate(self):
        self.ax.relim() 
        self.ax.autoscale_view()             
        
        plt.draw()
        plt.pause(self.pause)