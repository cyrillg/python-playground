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
from controller import Controller

f_sim = 1./20.
sim_speed = 1.
t_end = 200.

cart = Cart([-3., 5., pi/4])
path=[(0.0, 0.0),
      (4.0, 0.0),
      (3.0, -3.0),
      (1.0, -2.0),
      (-2.0, 0.0)]
controller = Controller(cart,
                        path=path)

Simulator(cart,
          controller,
          f_sim=f_sim,
          sim_speed=sim_speed)
