'''
Controller class for waypoint following

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class Controller:
    def __init__(self,
                 cart,
                 path=[(2.0, 0.0),
                       (3.0, -1.0),
                       (0.0, 0.0)]):
        self.path = path
        self.K = 0.15
        self.L = cart.L
        self.r = cart.r
        self.is_end = False

        self.wp_idx = 1
        self.current_wp = path[0]

    def generate_cmd(self, p):
        dist = sqrt(pow(p[0]-self.current_wp[0],2)+pow(p[1]-self.current_wp[1],2))
        if dist<0.2:
            if self.wp_idx<len(self.path):
                self.wp_idx += 1
                self.current_wp = self.path[self.wp_idx-1]
            else:
                self.is_end = True

        print(self.current_wp)
        if not self.is_end:
            wp = self.current_wp
            th_ref = arctan2(wp[1]-p[1],wp[0]-p[0])
            v = 0.1
            th_err = normalize(th_ref-p[2])
            print(rad2deg(th_err))
            w = self.K*th_err
            u0 = (2*v + self.L*w)/(2*self.r)
            u1 = (2*v - self.L*w)/(2*self.r)
        else:
            u0 = 0
            u1 = 0

        return (u0, u1)

    def is_end(self, p):
        dist = sqrt(pow(p[0]-self.current_wp[0],2)+pow(p[1]-self.current_wp[1],2))
        return dist<0.5