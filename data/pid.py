#!/usr/bin/python

from math import *
import random
from collections import deque
import matplotlib.pyplot as plt

class robot: 
    def __init__(self):
        self.power = 0.0 # -100 to 100
        self.speed_power_ratio = 1.0 / 60 # expected ratio

        # max is 1 m/s
        self.speed_queue = deque([0.0 for i in xrange(5)], maxlen=5) 

    def move(self, power_adj):
        res = robot()
        res.power = self.power
        res.speed_power_ratio = self.speed_power_ratio
        res.speed_queue = self.speed_queue 

        if abs(power_adj) < 0.001: # only update if greater than tol
            res.speed_queue.append(res.speed_queue[0])
        else:
            res.power += power_adj
            if res.power > 100.0:
                res.power = 100.0
            elif res.power < -100.0:
                res.power = -100.0

            res.speed_queue.append(res.power * res.speed_power_ratio)
        
        return res

    def __repr__(self):
        return '[speed=[%0.4f, %0.4f, %0.4f] power=%.4f]' \
            % (self.speed_queue[0], self.speed_queue[1], self.speed_queue[2], self.power)
        
def run(tau_p, tau_d, tau_i, bias):
    myrobot = robot()
    desired_speed = .75
    N = 100
    
    speeds_to_plot = []
    powers_to_plot = []

    i_cte = 0
    cte = desired_speed - myrobot.speed_queue[0]
    for i in xrange(N):
        speed_diff = desired_speed - myrobot.speed_queue[0]
        d_cte = (speed_diff - cte) / 1
        cte = speed_diff
        i_cte += cte

        # normalize power by multiplying with 100
        power_adj = bias + (tau_p * cte + tau_d * d_cte + tau_i * i_cte) * 100 
        myrobot = myrobot.move(power_adj)
        
        # print myrobot, power_adj / 100
        print speed_diff

        speeds_to_plot.append(myrobot.speed_queue[0])
        powers_to_plot.append(myrobot.power)

    plt.figure(1)
    plt.subplot(211)
    plt.plot(speeds_to_plot)
    plt.plot([desired_speed for i in xrange(100)], 'r--')
    
    plt.subplot(212)
    plt.plot(powers_to_plot)
    plt.show()

tau_p = 0.08
tau_d = 0.09
tau_i = 0.0001
bias = -0.1
run(tau_p, tau_d, tau_i, bias)
