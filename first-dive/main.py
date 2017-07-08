'''
Simulation of a kinematic model for a differential drive mobile robot

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *
from plant import Cart
from simulator import Simulator
from controller import OpenLoopCtrl, ClosedLoopCtrl

cart = Cart(p0=[-3., 5., -pi/4])

#--------------------------------------------------------------------------
# Open loop control
#commands = {5.: (1.0, 0.0),
#            10.: (2.0, -0.80),
#            15.: (0.0, 0.70),
#            20.: (-1.0, 0.0)}
#
#controller = OpenLoopCtrl(cart,
#                          reference=commands)
#
#Simulator(cart,
#          controller=controller)

#--------------------------------------------------------------------------
# Closed loop control
path=[(0.0, 0.0),
      (4.0, 0.0),
      (3.0, -3.0),
      (1.0, -2.0),
      (-2.0, 0.0)]

controller = ClosedLoopCtrl(cart,
                            reference=path)

Simulator(cart,
          controller=controller)
