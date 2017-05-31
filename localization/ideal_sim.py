'''
Simulation of the kinematics of a planar differential drive robot
in an ideal environment.

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
Please feel free to use and modify this, but keep the above information. Thanks!
'''

#!/usr/bin/env python

from lib import *

class Cart:
    '''Cart class

    p0: initial state [x, y, theta], with
      - x, y: planar position
      - theta: angular position in radians
    r: axle length
    '''
    def __init__(self,
                 p0=[0., 0., 0.],
                 r=1.):
        self.p = asarray(p0, dtype='float')
        self.r = r
        self.time_elapsed = 0.
        self.base_shape = [[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0],
                      [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]]

    def update_shape(self, col='darkblue'):
        '''Draws a cart from:
            - state x = [x, y, heading]
            - color col
            - scale factor r
        '''
        p = self.p.flatten()
        M = self.r*array(self.base_shape)
        M = transform_pattern(M, p[0], p[1], p[2])
        self.shape = M

    def dp_dt(self, p, u0, u1, t):
        '''derivative of the state'''
        dpdt = np.zeros_like(p)

        dpdt[0] = u0*cos(p[2])
        dpdt[1] = u0*sin(p[2])
        dpdt[2] = 0.

        return dpdt

    def step(self, dt):
        '''execute one time step of length dt and update state'''
        u = tuple([1.0, 0.])
        prev = self.p[0]
        self.p = odeint(self.dp_dt, self.p, [0, dt], args=u)[1]
        self.update_shape()
        print((self.p[0]-prev)/dt)
        self.time_elapsed += dt


if __name__=="__main__":
    cart = Cart([0., 0., 0.])
    dt = 1./10.

    fig = figure()
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                         xlim=(-10, 10), ylim=(-10, 10))
    ax.grid()

    line, = ax.plot([], [], lw=2)
    x_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    y_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
    th_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)

    def init():
        '''Initialize animation'''
        line.set_data([], [])
        x_text.set_text("")
        y_text.set_text("")
        th_text.set_text("")
        return line, x_text, y_text, th_text

    def animate(i):
        '''Animation step'''
        global cart, dt
        cart.step(dt)

        line.set_data(cart.shape[0], cart.shape[1])
        x_text.set_text("x = %.1f" % cart.p[0])
        y_text.set_text("y = %.1f" % cart.p[1])
        th_text.set_text("theta = %.1f" % rad2deg(cart.p[2]))
        return line, x_text, y_text, th_text

    t0 = time.time()
    animate(0)
    t1 = time.time()
    interval = dt - (t1 - t0)

    ani = FuncAnimation(fig, animate, frames=300,
                        interval=interval, blit=True, init_func=init)

    show()
