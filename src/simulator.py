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
from controller import OpenLoopCtrl
from observer import IdealObs

class Simulator:
    def __init__(self,
                 cart,
                 controller=None,
                 observer=None,
                 commands={},
                 t_end=10.,
                 f_sim=1./30.,
                 sim_speed=1.):

        self.cart = cart # Model to simulate

        if controller:
            self.controller = controller # Controller to the system
        else:
            self.controller = OpenLoopCtrl(self.cart,
                                           reference=commands)

        if observer:
            self.observer = observer
        else:
            self.observer = IdealObs(self.cart)

        self.sim_attr = {"speed": sim_speed,
                         "end": t_end,
                         "t": 0.,
                         "f_sim": f_sim} # Description of the simulation

        self.t = time.time()
        self.sim_t = 0.
        self.sim_end = False

        self.loop_timing = 0.

        # ----------------------------------------------------------------
        # Create display elements
        fig = figure()
        ax = fig.add_subplot(111,
                             aspect='equal',
                             autoscale_on=False,
                             xlim=(-10, 10),
                             ylim=(-7, 10))
        ax.grid()

        self.lines = (ax.plot([], [], color='b', lw=2)[0],
                      ax.plot([], [], color='r', lw=2)[0],
                      ax.text(0.75, 0.950, '', transform=ax.transAxes),
                      ax.text(0.02, 0.95, '', transform=ax.transAxes),
                      ax.text(0.02, 0.90, '', transform=ax.transAxes),
                      ax.text(0.02, 0.85, '', transform=ax.transAxes),
                      ax.text(0.02, 0.80, '', transform=ax.transAxes))

        if self.controller.type in ['closed-loop']:
            self.lines += (ax.scatter([e[0] for e in self.controller.path],
                                      [e[1] for e in self.controller.path],
                                      marker='o',
                                      s=[100]*len(self.controller.path)),)

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
            t1 = time.time()
            # ----------------------------------------------------------------
            # Generate current control inputs
            u = self.controller.generate_cmd(self.observer.p, self.sim_t)

            # ----------------------------------------------------------------
            # Compute the new system state
            t = time.time()
            sim_dt = self.sim_attr["speed"]*(t - self.t)

            self.cart.step(u, sim_dt)

            self.observer.update_est(self.cart.sense(), sim_dt)

            self.t = t
            self.sim_t += sim_dt

            # ----------------------------------------------------------------
            # Check if simulation is finished
            self.sim_end = self.controller.is_end or (self.sim_t
                                                      >self.sim_attr["end"])

            # ----------------------------------------------------------------
            # Update display
            self.lines[0].set_data(self.cart.shape[0],
                                   self.cart.shape[1])
            self.lines[1].set_data(self.observer.shape[0],
                                   self.observer.shape[1])
            self.lines[3].set_text("t = %.1f" % (self.sim_t))
            self.lines[4].set_text("x = %.2f" % self.cart.p[0])
            self.lines[5].set_text("y = %.2f" % self.cart.p[1])
            self.lines[6].set_text("theta = %.1f"%rad2deg(float(self.cart.p[2])))
            if self.controller.type in ['closed-loop']:
                new_colours = draw_path(self.controller.path,
                                        self.controller.wp_idx,
                                        self.sim_end)
                self.lines[7].set_color(new_colours)

            self.loop_timing = time.time() - t1
            if self.loop_timing>self.sim_attr["f_sim"]:
                print("WARNING: Loop timing: {}".format(t2-t1))
            return self.lines
