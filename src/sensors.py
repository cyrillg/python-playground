'''
Definition of the simulated robot sensors.

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class PerfectSensor:
    '''PerfectSensor class
    '''
    def __init__(self):
        self.current_readings = None

    def update_readings(self, p):
        self.current_readings = p
