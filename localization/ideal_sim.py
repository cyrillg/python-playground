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
        L: axle length
        r: wheel diameter
    '''
    def __init__(self,
                 p0=[0., 0., 0.],
                 L=1.,
                 r=0.3):
        self.p = asarray(p0, dtype='float').reshape((len(p0), 1))

        # Cart parameters
        self.L = L
        self.r = r

        self.t = 0.
        self.prev_x = 0.
        self.prev_t = 0.
        self.base_shape = [[0.25,-0.25,0,0,-0.25,-0.25,0,0,-0.25,0.25,
                                                            0,0,0.85,0.85,0],
                           [-0.5,-0.5,-0.5,-0.25,-0.25,0.25,0.25,0.5,0.5,
                                            0.5,0.5,0.25,0.125,-0.125,-0.25]]

    def update_shape(self, col='darkblue'):
        '''Update the ddrawing of the cart using:
            - state x = [x, y, heading]
            - color col
            - scale factor r
        '''
        p = self.p.flatten()
        M = self.L*array(self.base_shape)
        M = transform_pattern(M, p[0], p[1], p[2])
        self.shape = M

    def step(self, u, dt):
        '''Execute one time step of length dt and update state'''
        p = self.p.flatten()

        if u[0]==u[1]:
            dp = array([[self.r*u[0]*cos(p[2])],
                        [self.r*u[0]*sin(p[2])],
                        [0.]])
        else:
            R = self.L*(u[0] + u[1]) / (2*(u[0] - u[1]))
            a = self.r*(u[0] - u[1])/self.L

            IP = array([[ R*sin(p[2])],
                        [-R*cos(p[2])]])
            M = array([[cos(a*dt), -sin(a*dt), 0],
                       [sin(a*dt), cos(a*dt), 0],
                       [0, 0, 1]])
            U = vstack((IP,np.zeros(1)))
            dp = M @ U + vstack((-IP, array([a*dt])))

        self.p = self.p + dp
        self.p[2][0] = normalize(self.p[2][0])

        self.update_shape()
        self.t += dt

class Animation:
    def __init__(self, cart, dt, t_end, sim_speed):
        self.cart = cart

        fig = figure()
        ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                             xlim=(-10, 10), ylim=(-10, 10))
        ax.grid()

        self.line, = ax.plot([], [], lw=2)
        self.speed_text = ax.text(0.75, 0.950, '', transform=ax.transAxes)
        self.t_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        self.x_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
        self.y_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)
        self.th_text = ax.text(0.02, 0.80, '', transform=ax.transAxes)

        t0 = time.time()
        self.animate(0, dt, t_end)
        t1 = time.time()
        interval = dt/sim_speed*1000
        self.t_start = time.time()

        ani = FuncAnimation(fig,
                            self.animate,
                            frames=300,
                            fargs=(dt, t_end),
                            interval=interval,
                            blit=True,
                            init_func=self.init_anim)

        show()

    def init_anim(self):
        '''Initialize animation'''
        self.line.set_data([], [])
        self.speed_text.set_text("Speed: x{}".format(sim_speed))
        self.t_text.set_text("")
        self.x_text.set_text("")
        self.y_text.set_text("")
        self.th_text.set_text("")
        return self.line, self.t_text, self.x_text, self.y_text, self.th_text,

    def animate(self, i, dt, t_end):
        '''Animation step'''
        u = [-0.30, 0.30] # [w_r, w_l]
        if self.cart.t<t_end:
            self.cart.step(u, dt)

        self.line.set_data(cart.shape[0], cart.shape[1])
        self.t_text.set_text("t = %.1f" % cart.t)
        self.x_text.set_text("x = %.1f" % cart.p[0])
        self.y_text.set_text("y = %.1f" % cart.p[1])
        self.th_text.set_text("theta = %.1f" % rad2deg(float(cart.p[2])))
        return self.line, self.t_text, self.x_text, self.y_text, self.th_text,


if __name__=="__main__":
    cart = Cart([0., 0., 0.])
    dt = 1./30.
    sim_speed = 1.
    t_end = 30.

    Animation(cart, dt, t_end, sim_speed)
