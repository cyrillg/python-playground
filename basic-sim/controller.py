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
    def __init__(self, path):
        self.path = path
