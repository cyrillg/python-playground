#!/usr/bin/env python

from numpy import array, ones, cos, sin, pi, vstack
from matplotlib.pyplot import *

def draw_cart(x, col='darkblue', r=1):
    '''Draws a cart from:
        - state x = [x, y, heading]
        - color col
        - scale factor r
      '''
    x = x.flatten()
    cart_shape = [[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0],
                  [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]]
    M = r*array(cart_shape)
    M = transform_pattern(M, x[0], x[1], x[2])
    plot(M[0], M[1], col,2)

def transform_pattern(M, x, y, th):
    '''Performs the transformation on pattern M '''
    M1 = ones((1, len(M[1,:])))
    M2 = vstack((M, M1))
    R = array([[cos(th), -sin(th),x],
               [sin(th), cos(th),y]])
    return(R @ M2)
