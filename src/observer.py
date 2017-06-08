'''
Observer class

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class IdealObs:
    def __init__(self, cart):
        self.cart = cart
        self.p = cart.p
        self.base_shape = [[0.25,-0.25,0,0,-0.25,-0.25,0,0,-0.25,0.25,
                                                            0,0,0.85,0.85,0],
                           [-0.5,-0.5,-0.5,-0.25,-0.25,0.25,0.25,0.5,0.5,
                                            0.5,0.5,0.25,0.125,-0.125,-0.25]]
        self.shape = self.base_shape

    def update_shape(self):
        '''Update the ddrawing of the cart using:
            - state x = [x, y, heading]
            - scale factor r
        '''
        p = self.p.flatten()
        M = self.cart.L*array(self.base_shape)
        M = transform_pattern(M, p[0], p[1], p[2])
        self.shape = M

    def update_est(self, sensor_readings, sim_dt):
        self.p = sensor_readings[0]

        self.update_shape()
