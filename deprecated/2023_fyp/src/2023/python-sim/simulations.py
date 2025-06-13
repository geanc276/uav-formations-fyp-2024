import numpy as np
from formation import Formation

""" This file contains code instantiating different formations 
    and paths for leader paths to take """

""" Four Drones - leader drone stationary but rotating about its 
    own axis """
def init_formation1():
  
    initial_disp = {
        "1": np.array([0,0]),
        "2": np.array([2.5,13/3]),
        "3": np.array([5,0]),
        "4": np.array([2.5,13/9])
    }
    # if drone 4 is the leader drone of drone 1, then set row 1, column 4 to 1
    leader_table = np.array([[0,0,0,1], [0,0,0,1], [0,0,0,1], [0,0,0,0]])
    adj_matrix = np.array([[0,10,10,5.76],[10,0,10,5.76], [10,10,0,5.76], [5.76,5.76,5.76,0]]) # adjaceny matrix
    set_angle = np.array([[0,0,0,-2*np.pi/3],[0,0,0,0], [0,0,0,2*np.pi/3], [0,0,0,0]])
    k_p1 = 3*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # linear proportional gain 
    k_i1 = 3*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # linear integral gain
    k_p2 = 2.5*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # angular proportional gain 
    k_i2 = 0.1*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # angular integral gain

    leader_id = "4"

    # leader drone command velociy - path planning blackbox
    f = 0.05 # angular frequency 
    r = 15 # radius of circular path
    leader_vel = np.vectorize(lambda t: (
        np.array([0, 0]) 
    ))
    leader_w = lambda t : (
        0 if 0 <= t < 2 else
        0.8 if 2 <= t < 10 else
        0 if 10 <= t < 12 else
        -1.5
    )

    formation_data = np.array([adj_matrix, set_angle, k_p1, k_i1, k_p2, k_i2])
    G = Formation (formation_data, initial_disp, leader_id, leader_table, leader_vel, leader_w)
    T = 20

    return [G, T]


""" Four Drones - leader drone moving in linear path AND
    rotating about its own axis """
def init_formation2():
  
    initial_disp = {
        "1": np.array([0,0]),
        "2": np.array([2.5,13/3]),
        "3": np.array([5,0]),
        "4": np.array([2.5,13/9])
    }

    leader_table = np.array([[0,0,0,1], [0,0,0,1], [0,0,0,1], [0,0,0,0]])
    adj_matrix = np.array([[0,10,10,5.76],[10,0,10,5.76], [10,10,0,5.76], [5.76,5.76,5.76,0]]) # adjaceny matrix
    set_angle = np.array([[0,0,0,-2*np.pi/3],[0,0,0,0], [0,0,0,2*np.pi/3], [-2*np.pi/3,0,2*np.pi/3,0]])
    k_p1 = 2*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # linear proportional gain 
    k_i1 = 1*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # linear integral gain
    k_p2 = 2*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # angular proportional gain 
    k_i2 = 1*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # angular integral gain

    leader_id = "4"

    # leader drone command velociy - path planning blackbox
    f = 0.05 # angular frequency 
    r = 15 # radius of circular path
    leader_vel = np.vectorize(lambda t: (
        np.array([0, 0]) if 0 <= t < 4 else 
        np.array([4,4])
    ))
    leader_w = lambda t : (
        0 if 0 <= t < 2 else
        1
    )

    formation_data = np.array([adj_matrix, set_angle, k_p1, k_i1, k_p2, k_i2])
    G = Formation (formation_data, initial_disp, leader_id, leader_table, leader_vel, leader_w)
    T = 30

    return [G, T]


""" Four Drones - leader drone moving about curved trajectory.
    Angular velocity of leader drone no longer manual. Leader drone
    heading now matches leader drones commanded velocity vector.
     """
def init_formation3():
  
    initial_disp = {
        "1": np.array([0,0], dtype=np.float64),
        "2": np.array([2.5,13/3], dtype=np.float64),
        "3": np.array([5,0], dtype=np.float64),
        "4": np.array([2.5,13/9], dtype=np.float64)
    }

    leader_table = np.array([[0,0,0,1], [0,0,0,1], [0,0,0,1], [0,0,0,0]])
    adj_matrix = np.array([[0,10,10,5.76],[10,0,10,5.76], [10,10,0,5.76], [5.76,5.76,5.76,0]], dtype=np.float64) # adjaceny matrix
    set_angle = np.array([[0,0,0,-2*np.pi/3],[0,0,0,0], [0,0,0,2*np.pi/3], [-2*np.pi/3,0,2*np.pi/3,0]], dtype=np.float64)
    k_p1 = 3*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]], dtype=np.float64) # linear proportional gain 
    k_i1 = 1*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]], dtype=np.float64) # linear integral gain
    k_p2 = 2*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]], dtype=np.float64) # angular proportional gain 
    k_i2 = 1*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]], dtype=np.float64) # angular integral gain

    leader_id = "4"

    # leader drone command velociy - path planning blackbox
    f = 0.2 # angular frequency 
    leader_vel = np.vectorize(lambda t: (
        np.array([0, 0], dtype=np.float64) if 0 <= t < 4 else
        np.array([5,4*np.pi*f*np.cos(2*np.pi*f*t)], dtype=np.float64)
    ))

    formation_data = np.array([adj_matrix, set_angle, k_p1, k_i1, k_p2, k_i2])
    G = Formation (formation_data, initial_disp, leader_id, leader_table, leader_vel)
    T = 40

    return [G, T]


""" Four Drones - leader drone moving about circular trajectory.
    Angular velocity of leader drone no longer manual. Leader drone
    heading now matches leader drones commanded velocity vector. """
def init_formation4():
  
    initial_disp = {
        "1": np.array([0,0]),
        "2": np.array([2.5,13/3]),
        "3": np.array([5,0]),
        "4": np.array([2.5,13/9])
    }

    leader_table = np.array([[0,0,0,1], [0,0,0,1], [0,0,0,1], [0,0,0,0]])
    adj_matrix = np.array([[0,10,10,5.76],[10,0,10,5.76], [10,10,0,5.76], [5.76,5.76,5.76,0]]) # adjaceny matrix
    set_angle = np.array([[0,0,0,-2*np.pi/3],[0,0,0,0], [0,0,0,2*np.pi/3], [-2*np.pi/3,0,2*np.pi/3,0]])
    k_p1 = 3*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # linear proportional gain 
    k_i1 = 3*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # linear integral gain
    k_p2 = 3*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # angular proportional gain 
    k_i2 = 1*np.array([[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]) # angular integral gain

    leader_id = "4"

    # leader drone command velociy - path planning blackbox
    f = 0.05 # angular frequency 
    leader_vel = np.vectorize(lambda t: (
        np.array([0, 0]) if 0 <= t < 2 else
        np.array([-2*20*np.pi*f*np.sin(2*np.pi*f*t),2*20*np.pi*f*np.cos(2*np.pi*f*t)])
    ))

    formation_data = np.array([adj_matrix, set_angle, k_p1, k_i1, k_p2, k_i2])
    G = Formation (formation_data, initial_disp, leader_id, leader_table, leader_vel)
    T = 90

    return [G, T]


""" Four Drones - one drone is NOT connected to the leader drone. 
    This is very problematic!. Thus, all drones should be connected to 
    the leader drone, meaning control is technically NOT decentralized. 
    Unless data propogated throughout network. """
def init_formation5():

    initial_disp = {
        "1": np.array([0,10]),
        "2": np.array([6,10]),
        "3": np.array([-5,-5]),
        "4": np.array([5,-5])
    }

    leader_table = np.array([[0,0,0,0], [1,0,0,0], [1,0,0,0], [0,0,1,0]])
    adj_matrix = np.array([[0,5,5,0], [5,0,5*np.sqrt(2),5], [5,5*np.sqrt(2),0,5], [0,5,5,0]]) # adjaceny matrix
    set_angle = np.array([[0,0,0,0],[np.pi/2,0,0,0], [np.pi,0,0,0], [0,np.pi, np.pi/2, 0]])
    k_p1 = 2*np.array([[0,1,1,0],[1,0,1,1],[1,1,0,1],[0,1,1,0]]) # linear proportional gain 
    k_i1 = 0*np.array([[0,1,1,0],[1,0,1,1],[1,1,0,1],[0,1,1,0]]) # linear integral gain
    k_p2 = 2*np.array([[0,1,1,0],[1,0,1,1],[1,1,0,1],[0,1,1,0]]) # angular proportional gain 
    k_i2 = 0.1*np.array([[0,1,1,0],[1,0,1,1],[1,1,0,1],[0,1,1,0]]) # angular integral gain

    leader_id = "1"

    # leader drone command velociy - path planning blackbox
    f = 0.05 # angular frequency 
    leader_vel = np.vectorize(lambda t: (
        np.array([0, 0]) if 0 <= t < 5 else
        np.array([-2*15*np.pi*f*np.sin(2*np.pi*f*t),2*15*np.pi*f*np.cos(2*np.pi*f*t)])
    ))

    formation_data = np.array([adj_matrix, set_angle, k_p1, k_i1, k_p2, k_i2])
    G = Formation (formation_data, initial_disp, leader_id, leader_table, leader_vel)
    G.visualize ()
    T = 30

    return [G, T]
    
