'''
Simulator class for the kinematic model of differential drive mobile robot
described in cart_model.py

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from lib import *

class Simulator:
    def __init__(self, cart,
                 commands={5.: (0.30, -0.30)},
                 controller=None,
                 t_end=10.,
                 f_sim=1./30.,
                 sim_speed=1.):

        self.cart = cart # Model to simulate
        self.controller = controller # Controller to the system
        self.sim_attr = {"speed": sim_speed,
                         "end": t_end,
                         "t": 0} # Description of the simulation

        self.t = time.time()
        self.sim_t = 0.0
        self.sim_end = False

        # ----------------------------------------------------------------
        # Set up the command source (series of inputs or custom Controller)
        # If specified, a Controller has precedence and the series of
        # inputs will be discarded
        if not self.controller:
            self.commands = commands
            self.commands_ts = sorted(list(commands.keys()))
            self.current_cmd_end = self.commands_ts[0]
            self.cmd_idx = 0
            self.sim_attr["end"] = self.commands_ts[-1]

        # ----------------------------------------------------------------
        # Create display elements
        fig = figure()
        ax = fig.add_subplot(111,
                             aspect='equal',
                             autoscale_on=False,
                             xlim=(-10, 10),
                             ylim=(-5, 5))
        ax.grid()

        self.cart_line, = ax.plot([], [], lw=2)
        self.path_line = ax.scatter([e[0] for e in self.controller.path],
                                     [e[1] for e in self.controller.path])
        self.speed_text = ax.text(0.75, 0.950, '', transform=ax.transAxes)
        self.t_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        self.x_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
        self.y_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)
        self.th_text = ax.text(0.02, 0.80, '', transform=ax.transAxes)

        # ----------------------------------------------------------------
        # Launch simulation
        interval = f_sim*1000
        ani = FuncAnimation(fig,
                            self.step,
                            frames=300,
                            interval=interval,
                            blit=False)
        show()

    def step(self, i):
        '''Simulation step'''
        if not self.sim_end:
            # ----------------------------------------------------------------
            # Select current control inputs
            if self.controller:
                u = self.controller.generate_cmd(self.cart.p)
            else:
                if self.sim_t>self.current_cmd_end:
                    self.cmd_idx += 1
                    self.current_cmd_end = self.commands_ts[self.cmd_idx]
                u = self.commands[self.current_cmd_end]

            # ----------------------------------------------------------------
            # Compute the new system state
            t = time.time()
            sim_dt = self.sim_attr["speed"]*(t - self.t)

            self.cart.step(u, sim_dt)
            self.t = t
            self.sim_t += sim_dt

            # ----------------------------------------------------------------
            # Check if simulation is finished
            if self.controller:
                self.sim_end = self.controller.is_end
            else:
                self.sim_end = self.sim_t>=self.sim_attr["end"]

            # ----------------------------------------------------------------
            # Update display
            if self.controller:
                self.path_line.set_color(update_path_colours(self.controller.path,
                                                             self.controller.wp_idx,
                                                             self.sim_end))
            self.cart_line.set_data(self.cart.shape[0], self.cart.shape[1])
            self.t_text.set_text("t = %.1f" % (self.sim_t))
            self.x_text.set_text("x = %.2f" % self.cart.p[0])
            self.y_text.set_text("y = %.2f" % self.cart.p[1])
            self.th_text.set_text("theta = %.1f"%rad2deg(float(self.cart.p[2])))
            return self.cart_line,self.t_text,self.x_text,self.y_text,self.th_text,
