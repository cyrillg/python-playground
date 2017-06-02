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

class Simulator:
    def __init__(self, cart,
                 commands={5.: (0.30, -0.30)},
                 f_sim=1./30.,
                 sim_speed=1.):
        self.cart = cart

        fig = figure()
        ax = fig.add_subplot(111,
                             aspect='equal',
                             autoscale_on=False,
                             xlim=(-10, 10),
                             ylim=(-10, 10))
        ax.grid()

        self.line, = ax.plot([], [], lw=2)
        self.speed_text = ax.text(0.75, 0.950, '', transform=ax.transAxes)
        self.t_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        self.x_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
        self.y_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)
        self.th_text = ax.text(0.02, 0.80, '', transform=ax.transAxes)

        self.sim_speed = sim_speed
        interval = f_sim*1000
        self.t_start = time.time()
        self.t = self.t_start
        self.sim_time = 0

        self.commands = commands
        self.command_ts = sorted(list(commands.keys()))
        print(self.command_ts)
        self.current_cmd_end = self.command_ts[0]
        self.cmd_idx = 0
        self.last_cmd_end = self.command_ts[-1]

        ani = FuncAnimation(fig,
                            self.step,
                            frames=300,
                            interval=interval,
                            blit=True,
                            init_func=self.init_sim)

        show()

    def init_sim(self):
        '''Initialize animation'''
        self.line.set_data([], [])
        self.speed_text.set_text("Speed: x{}".format(sim_speed))
        self.t_text.set_text("")
        self.x_text.set_text("")
        self.y_text.set_text("")
        self.th_text.set_text("")
        return self.line, self.t_text, self.x_text, self.y_text, self.th_text,

    def step(self, i):
        '''Animation step'''
        print(self.cmd_idx)
        t = time.time()
        if self.sim_time<self.last_cmd_end:
            if self.sim_time>self.current_cmd_end:
                self.cmd_idx += 1
                self.current_cmd_end = self.command_ts[self.cmd_idx]
            u = self.commands[self.current_cmd_end]
            dt = t - self.t
            self.cart.step(u, self.sim_speed*dt)
            self.t = t
            self.sim_time += self.sim_speed*dt

        self.line.set_data(cart.shape[0], cart.shape[1])
        self.t_text.set_text("t = %.1f" % self.sim_time)
        self.x_text.set_text("x = %.1f" % cart.p[0])
        self.y_text.set_text("y = %.1f" % cart.p[1])
        self.th_text.set_text("theta = %.1f" % rad2deg(float(cart.p[2])))

        return self.line, self.t_text, self.x_text, self.y_text, self.th_text,


if __name__=="__main__":
    cart = Cart([0., 0., 0.])
    f_sim = 1./30.
    sim_speed = 1.
    t_end = 200.

    commands = {5.: (0.10, 0.10),
                10.: (0.20, 0.),
                15.: (-0.10, -0.10),
                20.: (-0.50, 0.50)}

    Simulator(cart, commands, f_sim, sim_speed)
