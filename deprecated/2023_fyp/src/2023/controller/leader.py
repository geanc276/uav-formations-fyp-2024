""" This file contains 2D velocity functions as a function of time which
    can be sent to the leader drone. These velocity functions will
    cause the leader drone and thus entire swarm to follow a particular 
    path i.e circular path """

import numpy as np
import matplotlib.pyplot as plt

def straight_path (t):
    """ returns command velocity for leader drone to travel purely translationally
        (in a straight line)"""
    
    vel = np.array([0,
                    0, 
                    0])

    return vel


def circular_path (t):  
    """ returns command velocity for leader drone travelling in a pure circular path 
        subject to parameters r and f """

    r = 10 # radius of path
    f = 0.025 # frequency of path: speed = 2*pi*r*f

    vel = np.array([2*np.pi*f*r*np.sin(2*np.pi*f*t),
                    2*np.pi*f*r*np.cos(2*np.pi*f*t), 
                    0])
    
    return vel


def sinusoidal_path (t):
    """ returns a command velocity for leader drone travelling sinusoidally in x but 
        linearly in y. """

    A = 2
    f = 0.1
    
    vel = A*np.array([2*np.pi*f*np.cos(2*np.pi*f*t),
                    3, 
                    0])
    
    return vel



def figure8 (t):
    """ returns a command velocity for leader drone for travelling in a figure 8"""

    A = 25
    f = 0.03
    w = 2*np.pi*f
    
    vel = np.array([A*w*np.cos(w*t),
                    A*w*np.cos(2*w*t), 
                    0])

    return vel


def leader_rotation ():
    """ returns leader correction which causes leader to purely rotate on it's own axis
        with some angular velocity, omega_z"""
    
    omega_z = 0.5
    total_correction = np.zeros((6,))
    total_correction[-1] = omega_z

    return total_correction


def arbitrary_path (t):
    """ piecewise leader velocity function can be defined by the user such that
        the leader drone can move in any arbitrary path """
    
    vel = lambda t : (
        np.array([0,
                  0,
                  0]) if 0 <= t < 10 else 
        np.array([4,
                  t-10,
                  0]) if 10 <= t < 14 else 
        np.array([2*np.pi*0.1*np.cos(2*np.pi*0.1*t),
                  4,
                  0]) 
    ) 

    return vel(t)


def manual_path (t):
    """ piecewise leader correction function can be defined by the user such that
        the leader drone can move in any arbitrary path - this functin also includes
        manual setting of angular velocity """

    total_correction = lambda t : (
        np.array([0,
                  0,
                  0,
                  0,
                  0,
                  0.5]) if 0 <= t < 10 else 
        np.array([t-10,
                  t-10,
                  0,
                  0,
                  0,
                  0.5]) if 10 <= t < 14 else 
        np.array([4.0,
                  4.0,
                  0,
                  0,
                  0,
                  0]) 
    ) 

    return total_correction(t)


""" PLOTTING FUNCTIONS FOR LEADER PATH AND VELOCITY """
def plot_paths(velocity_function, duration=10, num_points=1000):
    """ plots leader expected path for given velocity function 
        via integration """

    # Initialize empty arrays for velocity and position
    vel = np.zeros((3, num_points))
    pos = np.zeros((3, num_points))
    t = np.linspace(0, duration, num_points)
   
    # Calculate velocity vectors for each time step
    for i, ti in enumerate(t):
        vel[:, i] = velocity_function(ti)
    
    # Integrate velocity vectors to get position vectors
    for i in range(1, num_points):
        delta_t = t[i] - t[i-1]
        pos[:, i] = pos[:, i-1] + vel[:, i] * delta_t
    
    # Plot the path in 2D (x-y plane) with a grid
    plt.figure(figsize=(8, 6))
    plt.plot(pos[0], pos[1], label='Path', color='blue')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Path Traced by Velocity Function (2D)')
    plt.grid(True)  # Add a grid to the plot
    plt.axis('equal')  # Equal aspect ratio to maintain proportions
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_velocities(velocity_function, duration=10, num_points=1000):
    """ plots leader velocities, x, y and absolute for given velocity
        function"""
    
    # initialize empty arrays for velocity
    vel = np.zeros((3, num_points))
    t = np.linspace(0, duration, num_points)
    
    # valculate velocity vectors for each time step
    for i, ti in enumerate(t):
        vel[:, i] = velocity_function(ti)
    
    # extract velocity components
    x_vel = vel[0]
    y_vel = vel[1]
    abs_vel = np.linalg.norm(vel[:2], axis=0)  # Use only x and y components
    
    # plot
    plt.figure(figsize=(10, 6))
    plt.plot(t, x_vel, label='X Velocity', color='red')
    plt.plot(t, y_vel, label='Y Velocity', color='green')
    plt.plot(t, abs_vel, label='Absolute Velocity', color='blue')
    plt.xlabel('Time')
    plt.ylabel('Velocity (m/s)')
    plt.title('X and Y Velocities, Absolute Velocity')
    plt.legend()
    plt.tight_layout()
    plt.grid(True)
    plt.show()


if __name__ == "__main__": 
    duration = 50
    velocity_function = figure8
    plot_paths(velocity_function, duration, duration*50  )
    plot_velocities(velocity_function, duration, duration*50  )
