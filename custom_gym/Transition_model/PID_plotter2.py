from Pid_plotter import PIDPlotter
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PIDPlotter():

    def __init__(self):
    # Create figure for plotting
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.xs = []
        self.ys = []


        # This function is called periodically from FuncAnimation
        def _animate(i,xs,ys):

            

            # Add x and y to lists
            xs.append(x)
            ys.append(y)

            # # Limit x and y lists to 20 items
            # xs = xs[-20:]
            # ys = ys[-20:]

            # Draw x and y lists
            self.ax.clear()
            self.ax.plot(self.xs, self.ys)

            # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.subplots_adjust(bottom=0.30)
            plt.title('PID')
            plt.ylabel('Velocity (m/s)')


        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(fig, _animate, fargs=(self.xs, ys), interval=1000)
        plt.show()
