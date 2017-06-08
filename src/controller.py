'''
Controller class for waypoint following

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class OpenLoopCtrl:
    def __init__(self,
                 cart,
                 reference={5.: (0.30, 0.30),
                            10.: (1.20, 0.30),
                            15.: (-0.30, -0.30),
                            20.: (-1.50, 1.50)}):
        self.type = 'open-loop'
        self.L = cart.L
        self.r = cart.r
        self.is_end = False

        if len(reference)>0:
            self.commands = reference
            self.commands_ts = sorted(list(self.commands.keys()))
            self.current_cmd_end = self.commands_ts[0]
            self.cmd_idx = 0
            self.t_end = self.commands_ts[-1]
        else:
            self.t_end = 0.0

    def generate_cmd(self, p, t):
        if t<self.t_end:
            if t>self.current_cmd_end:
                self.cmd_idx += 1
                self.current_cmd_end = self.commands_ts[self.cmd_idx]
            u = self.commands[self.current_cmd_end]
        else:
            u = (0, 0)
            self.is_end = True

        return u


class ClosedLoopCtrl:
    def __init__(self,
                 cart,
                 reference=[(2.0, 0.0),
                            (3.0, -1.0),
                            (0.0, 0.0)]):
        self.type = 'closed-loop'
        self.path = reference
        self.K = 2.
        self.v = 2.
        self.L = cart.L
        self.r = cart.r
        self.is_end = False

        self.wp_idx = 1
        self.current_wp = self.path[0]

        print("\nController launched")
        print("  Path composed of {} waypoints".format(len(self.path)))
        print("\nInitial target: {}. {}".format(self.wp_idx-1,
                                              self.current_wp))

    def generate_cmd(self, p, t):
        dist = sqrt(pow(p[0]-self.current_wp[0],2)
                    +pow(p[1]-self.current_wp[1],2))
        if dist<0.2:
            if self.wp_idx<len(self.path):
                self.wp_idx += 1
                self.current_wp = self.path[self.wp_idx-1]
                print("New target: {}. {}".format(self.wp_idx-1,
                                                  self.current_wp))
            elif not self.is_end:
                self.is_end = True
                print("\nFinal target reached\n")

        if not self.is_end:
            wp = self.current_wp
            th_ref = arctan2(wp[1]-p[1],wp[0]-p[0])
            v = self.v
            th_err = normalize(th_ref-p[2])
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
