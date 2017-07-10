'''
Definition of controller classes

Controllers gather the functions used to generate the control inputs
fed to the Model

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class OpenLoopCtrl:
    ''' Open loop controller definition

        Inputs:
          - cart: Model object. Model parameters are used in the
                  command generation
          - reference: sequence of command inputs (w_r, w_l) where w_r, w_l are
                       respectively the right and left wheel angular speeds. Must
                       be specified as a dictionnary of tuples indexed with the
                       time each specific command ends.
    '''
    def __init__(self,
                 cart,
                 reference={5.: (0.30, 0.30),
                            10.: (1.20, 0.30),
                            15.: (-0.30, -0.30),
                            20.: (-1.50, 1.50)}):
        self.type = "open-loop"
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

    def transform(self, v, w):
        ''' Transform linear and angular speed into wheel angular speeds
        '''
        u0 = (2*v + self.L*w) / (2*self.r)
        u1 = (2*v - self.L*w) / (2*self.r)

        return (u0, u1)

    def generate_cmd(self, p, t):
        if t<self.t_end:
            if t>self.current_cmd_end:
                self.cmd_idx += 1
                self.current_cmd_end = self.commands_ts[self.cmd_idx]
            u = self.transform(self.commands[self.current_cmd_end][0],
                               self.commands[self.current_cmd_end][1])
        else:
            u = (0, 0)
            self.is_end = True

        return u


class ClosedLoopCtrl:
    ''' Closed loop controller definition

        Inputs:
          - cart: Model object. Model parameters are used in the
                  command generation
          - reference: sequence of (x, y) waypoints. Must be specified as a
                       list of tuples.
    '''
    def __init__(self,
                 cart,
                 reference=[(2.0, 0.0),
                            (3.0, -1.0),
                            (0.0, 0.0)]):
        self.type = "closed-loop"
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
        ''' Main function to generate the current wheel angular speed inputs
        '''
        self.current_wp = self.supervise(p)

        if not self.is_end:
            th_err = self.LOS(p, self.current_wp)

            v = self.v
            w = self.P(self.K, th_err)

        else:
            v = 0
            w = 0
        (u0, u1) = self.transform(v, w)

        return (u0, u1)

    def supervise(self, p):
        ''' Supervisor handling waypoint switching and simulation end
        '''
        current_wp = self.current_wp

        dist = sqrt(pow(p[0]-current_wp[0],2)
                    +pow(p[1]-current_wp[1],2))
        if dist<0.2:
            if self.wp_idx<len(self.path):
                self.wp_idx += 1
                current_wp = self.path[self.wp_idx-1]
                print("New target: {}. {}".format(self.wp_idx-1,
                                                  self.current_wp))
            elif not self.is_end:
                self.is_end = True
                current_wp = None
                print("\nFinal target reached\n")

        return current_wp

    def LOS(self, p, wp):
        ''' Guidance law to generate the heading reference
        '''
        th_ref = arctan2(wp[1]-p[1],wp[0]-p[0])
        th_err = normalize(th_ref-p[2])

        return th_err

    def P(self, K, err):
        ''' P controller to generate the angular speed command
        '''
        return K*err

    def transform(self, v, w):
        ''' Transform linear and angular speed into wheel angular speeds
        '''
        u0 = (2*v + self.L*w) / (2*self.r)
        u1 = (2*v - self.L*w) / (2*self.r)

        return (u0, u1)
