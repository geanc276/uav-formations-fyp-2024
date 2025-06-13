import numpy as np
from visualize import *
from formation import *
from control import *
from simulations import * 

def main ():
    """ runs simulation for drone swarm using control system. Returns solved path 
    trajectories for each drone in x and y. """

    formation, T = init_formation4 ()
    dt = 0.1
    time_steps = int(T/dt) 

    # perform 10 second simulation 
    for i in range(0,time_steps-1):
        t = dt*i # current time 
        formation = control_iteration (formation, dt, t) # loop through control loop each delta t
        formation = calc_new_pos (formation, dt)

    # visualize simulation 
    xlim = None
    ylim = None

    # plots simulation results
    visualize_swarm_2D (formation.vertices, dt, xlim, ylim)
    plot_error (formation, T, dt)
    plot_command_vel (formation.vertices, T, dt)
    plot_paths_2D (formation.vertices)
9
if __name__ == '__main__':
    main ()