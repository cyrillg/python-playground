'''
Definition of observer classes

Observers gather the functions used to compute the current estimate of the
state

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class IdealObs:
    ''' Definition of an ideal observer

        Detail:
          This controller's estimate matches the actual state perfectly
    '''
    def __init__(self, cart):
        self.cart = cart
        self.p = cart.p
        self.L = cart.L
        self.base_shape = cart.base_shape
        self.shape = self.base_shape

    def update_shape(self):
        ''' Update the drawing of the cart
        '''
        p = self.p.flatten()
        M = self.L*array(self.base_shape)
        M = transform_pattern(M, p[0], p[1], p[2])
        self.shape = M

    def update_est(self, sensor_readings, dt):
        ''' Provide the new estimate of the system state

            Inputs:
              - sensor_readings: current sensor readings
              - dt: time passed since last estimation
        '''
        self.p = sensor_readings[0]

        self.update_shape()
