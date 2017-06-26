'''
Simulator class for the kinematic model of differential drive mobile robot
described in model.py

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
    ''' Class executing the simulation of the specified parts

        Inputs:
          - cart: Model object, decribing the plant
          - controller: Object used to generate the commands to follow a given
                        reference (wheel speed sequence, path...)
          - observer: state estimator of the system
          - sim_timeout: timeout after which the simulation will stop, whether or
                         not the controller has reached the final target
          - p_sim: simulation step
          - sim_speed: simulation speed
    '''
    def __init__(self,
                 cart,
                 controller=None,
                 observer=None,
                 sim_timeout=300.,
                 p_sim=1./20.,
                 sim_speed=1.):


        # ----------------------------------------------------------------
        # Set up model, controller, and observer

        self.cart = cart # Model to simulate

        if controller:
            self.controller = controller # Specified
        else:
            self.controller = OpenLoopCtrl(self.cart) # Default

        if observer:
            self.observer = observer # Specified
        else:
            self.observer = IdealObs(self.cart) # Default

        # ----------------------------------------------------------------
        # Simulation attributes
        self.sim_attr = {"speed": sim_speed,
                         "timeout": sim_timeout,
                         "p_sim": p_sim,
                         }

        self.t = time.time()
        self.sim_t = 0.
        self.sim_complete = False
        self.loop_dt = 0.

        # ----------------------------------------------------------------
        # Create display elements
        fig = figure()
        ax = fig.add_subplot(111,
                             aspect="equal",
                             autoscale_on=False,
                             xlim=(-10, 10),
                             ylim=(-7, 10))
        ax.grid()

        self.lines = (ax.plot([], [], color="b", lw=2)[0],
                      ax.plot([], [], color="r", lw=2)[0],
                      ax.text(0.75, 0.950, "", transform=ax.transAxes),
                      ax.text(0.02, 0.95, "", transform=ax.transAxes),
                      ax.text(0.02, 0.90, "", transform=ax.transAxes),
                      ax.text(0.02, 0.85, "", transform=ax.transAxes),
                      ax.text(0.02, 0.80, "", transform=ax.transAxes))

        if self.controller.type in ["closed-loop"]:
            self.lines += (ax.scatter([e[0] for e in self.controller.path],
                                      [e[1] for e in self.controller.path],
                                      marker="o",
                                      s=[100]*len(self.controller.path)),)

        # ----------------------------------------------------------------
        # Launch simulation
        interval = p_sim*1000
        ani = FuncAnimation(fig,
                            self.step,
                            frames=300,
                            interval=interval,
                            blit=False)
        show()

    def step(self, i):
        ''' Simulation step
        '''
        if not self.sim_complete:
            t1 = time.time()

            # ----------------------------------------------------------------
            # Generate current control inputs
            u = self.controller.generate_cmd(self.observer.p, self.sim_t)

            # ----------------------------------------------------------------
            # Compute the new system state
            t = time.time()
            sim_dt = self.sim_attr["speed"]*(t - self.t)

            self.cart.step(u, sim_dt) # Plant step
            self.observer.update_est(self.cart.sense(),
                                     sim_dt) # New state estimate
            self.t = t
            self.sim_t += sim_dt

            # ----------------------------------------------------------------
            # Check if simulation is finished
            self.sim_complete = (self.controller.is_end
                                 or (self.sim_t>self.sim_attr["timeout"]))

            # ----------------------------------------------------------------
            # Update display
            self.lines[0].set_data(self.cart.shape[0],
                                   self.cart.shape[1])
            self.lines[1].set_data(self.observer.shape[0],
                                   self.observer.shape[1])
            self.lines[3].set_text("t = %.1f" % (self.sim_t))
            self.lines[4].set_text("x = %.2f" % self.cart.p[0])
            self.lines[5].set_text("y = %.2f" % self.cart.p[1])
            self.lines[6].set_text("theta = %.1f"%rad2deg(self.cart.p[2]))
            if self.controller.type in ["closed-loop"]:
                new_colours = draw_path(self.controller.path,
                                        self.controller.wp_idx,
                                        self.sim_complete)
                self.lines[7].set_color(new_colours)

            # ----------------------------------------------------------------
            # Check for jam in the simulation
            self.loop_dt = time.time() - t1
            if self.loop_dt>self.sim_attr["p_sim"]:
                print("/!\ Loop duration exceeds timestep: {}".format(t2-t1))

            return self.lines
