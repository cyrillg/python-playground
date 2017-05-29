#!/usr/bin/env python

from lib import *

x = array([[4, 6, pi]])

fig = figure(0)
ax = fig.add_subplot(111, aspect='equal')
draw_cart(x,col='red',r=10)
show()
