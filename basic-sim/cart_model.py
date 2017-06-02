'''
Kinematic model of a planar differential drive robot.

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class Cart:
    '''Cart class

        p0: initial state [x, y, theta], with
          - x, y: planar position
          - theta: angular position in radians
        L: axle length
        r: wheel diameter
    '''
    def __init__(self,
                 p0=[0., 0., 0.],
                 L=1.,
                 r=0.3):
        self.p = asarray(p0, dtype='float').reshape((len(p0), 1))

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

    def update_shape(self, col='darkblue'):
        '''Update the ddrawing of the cart using:
            - state x = [x, y, heading]
            - color col
            - scale factor r
        '''
        p = self.p.flatten()
        M = self.L*array(self.base_shape)
        M = transform_pattern(M, p[0], p[1], p[2])
        self.shape = M

    def step(self, u, dt):
        '''Execute one time step of length dt and update state'''
        p = self.p.flatten()

        if u[0]==u[1]:
            dp = array([[self.r*u[0]*cos(p[2])],
                        [self.r*u[0]*sin(p[2])],
                        [0.]])
        else:
            R = self.L*(u[0] + u[1]) / (2*(u[0] - u[1]))
            a = self.r*(u[0] - u[1])/self.L

            IP = array([[ R*sin(p[2])],
                        [-R*cos(p[2])]])
            M = array([[cos(a*dt), -sin(a*dt), 0],
                       [sin(a*dt), cos(a*dt), 0],
                       [0, 0, 1]])
            U = vstack((IP,np.zeros(1)))
            dp = M @ U + vstack((-IP, array([a*dt])))

        self.p = self.p + dp
        self.p[2][0] = normalize(self.p[2][0])

        self.update_shape()
