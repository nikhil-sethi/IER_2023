from scipy.spatial import distance
import matplotlib.pyplot as plt
import numpy as np
import pickle
from itertools import combinations
from tools.vector_math import Vector2

class Graphic(object):

    def init_func(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # axis dimensions
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])

        # uav_list
        self.uavs = []
        self.di = list()  # np.zeros((N,N))
        
        # generate meshgrid

    # function to be followed on every button click
    def onclick(self,event):

        self.uavs.append(Vector2([event.ydata, event.xdata]))
        # unzip x_y
        temp_y, temp_x = zip(*self.uavs)
        plt.plot(temp_x, temp_y)
        plt.plot(event.xdata, event.ydata, color='red', marker='o', markersize=9)

        self.fig.canvas.draw()


    def display(self):
        self.init_func()
        # plt.scatter(self.x, self.y)
        x, y, arrow_length = 0.5, 0.5, 0.15
        self.ax.annotate('N', xy=(x, y), xytext=(x, y-arrow_length),
            arrowprops=dict(facecolor='red', width=5, headwidth=15),
            ha='center', va='center', fontsize=20,
            xycoords=self.ax.transAxes)
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        plt.show()
        # d_matrix = distance.cdist(self.uavs, self.uavs, 'euclidean')
        N = len(self.uavs)
        v_mat = [[Vector2([0.,0.]) for _ in range(N)] for _ in range(N)]
        for a, b in combinations(enumerate(self.uavs),2):
            v_mat[a[0]][b[0]] = a[1] - b[1]
            v_mat[b[0]][a[0]] = b[1] - a[1]
        if plt.close() == True:
            print("OKAY")
        plt.close()
        return v_mat


if __name__=='__main__':
    formation  = Graphic().display()
    print("formation", formation)

    with open("formation", "wb") as form_file:
    	pickle.dump(formation, form_file)
