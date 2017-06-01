#!/usr/bin/env python

from numpy import array, ones, cos, sin, pi, vstack, asarray, rad2deg, dot
from scipy.integrate import odeint
from matplotlib.pyplot import *
from matplotlib.animation import *
import time

def transform_pattern(M, x, y, th):
    '''Performs the transformation on pattern M '''
    M1 = ones((1, len(M[1,:])))
    M2 = vstack((M, M1))
    R = array([[cos(th), -sin(th),x],
               [sin(th), cos(th),y]])
    return(R @ M2)
