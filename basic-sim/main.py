'''
Simulation of a kinematic model for differential drive mobile robot

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from cart_model import Cart
from simulator import Simulator

f_sim = 1./20.
sim_speed = 6.
t_end = 200.

cart = Cart([0., 0., 0.])
commands = {5.: (0.10, 0.10),
            10.: (0.40, 0.10),
            15.: (-0.10, -0.10),
            20.: (-0.50, 0.50)}

Simulator(cart,
          commands=commands,
          f_sim=f_sim,
          sim_speed=sim_speed)
