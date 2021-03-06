'''
Kinematic model of a planar differential drive robot.

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *
from sensor import PerfectSensor

class Cart:
    ''' Cart class

        Inputs:
          - p0: initial state [x, y, theta], with
             * x, y: planar position
             * theta: angular position in radians
          - L: axle length
          - r: wheel diameter
    '''
    def __init__(self,
                 p0=[0., 0., 0.],
                 L=1.0,
                 r=1.0):
        self.p = asarray(p0, dtype="float")
        self.prev = asarray(p0, dtype="float")

        # Sensors
        self.sensors = [PerfectSensor()]

        # Cart parameters
        self.L = L
        self.r = r

        self.prev_x = 0.
        self.prev_t = 0.
        self.base_shape = [[0.25,-0.25,0,0,-0.25,-0.25,0,0,-0.25,0.25,
                                                            0,0,0.85,0.85,0],
                           [-0.5,-0.5,-0.5,-0.25,-0.25,0.25,0.25,0.5,0.5,
                                            0.5,0.5,0.25,0.125,-0.125,-0.25]]
        self.shape = self.base_shape

    def update_shape(self):
        ''' Update the drawing of the cart

            Inputs:
              - state x = [x, y, heading]
              - scale factor r
        '''
        p = self.p.flatten()
        M = self.L*array(self.base_shape)
        M = transform_pattern(M, p[0], p[1], p[2])
        self.shape = M

    def dp_dt(self, p, t, u0, u1):
        ''' Derivative of the state

            Inputs:
              - p: state of the cart
              - u0, u1: respectively right and left wheel angular speeds
        '''
        dpdt = np.zeros_like(p)

        v = self.r/2 * (u0 + u1)
        w = self.r*(u0 - u1)/self.L

        dpdt[0] = v*cos(p[2])
        dpdt[1] = v*sin(p[2])
        dpdt[2] = w

        return dpdt

    def step(self, u, dt):
        ''' Execute one time step of length dt and update state

            Inputs:
              - u: current control inputs
              - dt: duration u is applied
        '''
        self.p_prev = self.p

        self.p = odeint(self.dp_dt, self.p, [0, dt], args=u)[1]
        self.p[2] = normalize(self.p[2])

        self.update_shape()

    def sense(self):
        ''' Gather current readings from the model's sensors
        '''
        for sensor in self.sensors:
            sensor.update_readings(self.p)

        return [sensor.current_readings for sensor in self.sensors]
