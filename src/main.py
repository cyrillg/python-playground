'''
Simulation of a kinematic model for differential drive mobile robot

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *
from cart_model import Cart
from simulator import Simulator
from controller import OpenLoopCtrl, ClosedLoopCtrl

f_sim = 1./30.
sim_speed = 1.
t_end = 200.

cart = Cart([-3., 5., -pi/4])

commands = {5.: (1.0, 1.00),
            10.: (0.50, 1.0),
            15.: (-0.70, 0.70),
            20.: (0.50, 0.50)}
controller = OpenLoopCtrl(cart,
                          reference=commands)

#path=[(0.0, 0.0),
#      (4.0, 0.0),
#      (3.0, -3.0),
#      (1.0, -2.0),
#      (-2.0, 0.0)]
#controller = ClosedLoopCtrl(cart,
#                            reference=path)

Simulator(cart,
          f_sim=f_sim,
          t_end=t_end,
          controller=controller,
          sim_speed=sim_speed)
