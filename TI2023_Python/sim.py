import numpy as np
from math import sqrt, exp, log
import matplotlib.pyplot as plt
from matplotlib.backend_bases import LocationEvent
from matplotlib.patches import Rectangle
from warnings import warn

LENGTH = 300
WIDTH = 300
HALF_SQUARE = LENGTH / 2 + 60
UNIT = 25
V_VOICE = 340
CLK_FREQ = 84000000
Q1 = np.array([HALF_SQUARE, HALF_SQUARE])
Q2 = np.array([-HALF_SQUARE, HALF_SQUARE])
Q3 = np.array([-HALF_SQUARE, -HALF_SQUARE])
Q4 = np.array([HALF_SQUARE, -HALF_SQUARE])
MICROPHONES = np.array([Q1, Q2, Q3, Q4])
POINT_DIST = np.zeros(4)
CORRECT_POINT_DIST = np.zeros(4)
O = np.array([0, 0])
G_VECTOR = np.zeros((4, 2))
RW, RH = 15., 15.
P = None
total_steps = 50
steps = 15
P_skip_num = 0
drop_point = True

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
fig, ax = plt.subplots()

def gradient_descent(pi:np.ndarray, step:int):
    sum = 0
    pi1 = pi
    gradients_list = np.zeros((4, 2))
    for name, p in enumerate(MICROPHONES):
        POINT_DIST[name] = dist(p, pi) - dist(MICROPHONES[P_skip_num], pi) - (CORRECT_POINT_DIST[name] - CORRECT_POINT_DIST[P_skip_num])
        sum += abs(POINT_DIST[name])
    for name, p in enumerate(MICROPHONES):
        POINT_DIST[name] /= sum
        gradients_list[name] = G_VECTOR[name] * POINT_DIST[name] * (steps + sum / 20) *  (1 - log(1 + exp(step / 5 - 10)) / 0.65)#  (exp(-step / 8) + 0.5)
        pi1 += gradients_list[name]
    return pi1

def gradient_descent_wrapper():
    global P_skip_num, G_VECTOR
    P_skip_num = np.argmin(CORRECT_POINT_DIST)
    P0 = MICROPHONES[P_skip_num] / HALF_SQUARE * LENGTH / 4
    if drop_point:
        ax.scatter(P0[0], P0[1], 90, "y", "+", label="初始点")
    else:
        ax.scatter(P0[0], P0[1], 90, "y", "+")
    Pi = P0
    for i in range(4):
        G_VECTOR[i] = MICROPHONES[i] - MICROPHONES[P_skip_num]
        if i != P_skip_num:
            G_VECTOR[i] /= dist(O, G_VECTOR[i]) * sqrt(3)
    for step in range(total_steps):
        Pi = gradient_descent(Pi, step)
        # if step > total_steps - 5:
        #     print(f"{step}th dist:{dist(Pi, P)}")
        if step == 0 and drop_point:
            ax.scatter(Pi[0], Pi[1], s=6, c="b", marker="*", label="下降路径点")
        else:
            ax.scatter(Pi[0], Pi[1], s=6, c="b", marker="*")
        plt.draw()
    print(f"final:{Pi}")

def draw_target():
    ax.set_aspect(1.)
    ax.scatter(Q1[0], Q1[1], 30, "r", "o", label="麦克风")
    ax.scatter(Q2[0], Q2[1], 30, "r", "o")
    ax.scatter(Q3[0], Q3[1], 30, "r", "o")
    ax.scatter(Q4[0], Q4[1], 30, "r", "o")
    for i in range(LENGTH // UNIT + 1):
        ax.hlines(i * UNIT - LENGTH / 2, -LENGTH / 2, LENGTH / 2, "black", "dashdot")
        ax.vlines(i * UNIT - WIDTH / 2, -WIDTH / 2, WIDTH / 2, "black", "dashdot")

def dist(p1:np.ndarray, p2:np.ndarray):
    return np.sqrt(np.sum(np.power(p2 - p1, 2)))

def onclick(event:LocationEvent):
    global P, drop_point
    if event.inaxes:
        P = np.array([event.xdata, event.ydata])
        print(f"origin:{P}")
        if drop_point:
            artists_object = Rectangle(xy=[P[0], P[1] - RH / sqrt(2)], width=RW, height=RH, angle=45, facecolor="g", label="声源")
        else:
            artists_object = Rectangle(xy=[P[0], P[1] - RH / sqrt(2)], width=RW, height=RH, angle=45, facecolor="g")
        ax.add_artist(artists_object)
        plt.legend(loc="lower center", ncols=2)
        plt.draw()
        for index, p in enumerate(MICROPHONES):
            CORRECT_POINT_DIST[index] = dist(p, P)
            print(f"to {index}:{CORRECT_POINT_DIST[index]}")
        if abs(P[0]) > LENGTH / 2 or abs(P[1]) > WIDTH / 2:
            warn("warning: out of bound!")
        else:
            gradient_descent_wrapper()
        print()
        drop_point = False
    else:
        ax.cla()
        draw_target()
        print("clear")
        plt.draw()
        drop_point = True

cid = fig.canvas.mpl_connect('button_press_event', onclick)
draw_target()
plt.show()