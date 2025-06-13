""" control loop for drone formation """

import numpy as np
from numpy import linalg as LA
from FSM import MyStateMachine
import time 

CLAMP_VELOCITY = 12
MAX_ACC = 30

def angle_wrap (theta):
    """ returns angle in domain [-180, 180] deg """
    while theta > np.pi:
        theta -= 2*np.pi
    while theta < -np.pi:
        theta += 2*np.pi

    return theta


def alpha (edge, v_ji):
    """ weighting function based on error from current state 
    to desired state (setpoint) """

    return 1 - (edge.weight/LA.norm(v_ji)) 


def calc_new_pos (formation, dt):
    """ calculate new position of each drone formation based on commanded velocity
    calculated by control sytem """

    for uav in formation.vertices:
        # kinematics to update drone position 
        new_pos = uav.disp + (uav.vel * dt)
        uav.disp = new_pos
        uav.path_x.append(new_pos[0])
        uav.path_y.append(new_pos[1])
        uav.path_z.append(0)

        # update heading of drones
        uav.current_heading += uav.w*dt
        uav.heading_history.append(uav.current_heading)

    return formation


def follower_control (uav, formation, dt):
    """ feedback control for follower drones """

    x_i_dot = np.zeros(2) # command velocity for uav

    for edge in uav.edges:
        # perform gain scheduling based on the formation state 
        if formation.state_machine.state == 'MOVING':
            k_i1 = edge.k_i1
        else:
            k_i1 = 0

        # PI linear control
        neighbour_uav = formation.get_vertex (edge.to_vertex.id)
        v_ji = neighbour_uav.disp - uav.disp
        edge.error = alpha (edge, v_ji)*v_ji
        edge.error_derivative = (edge.error - edge.previous_error) / dt

        try:
            edge.error_int, edge.previous_error = integrate(edge.error,dt,edge.previous_error,edge.error_int) 
        except:
            edge.error_int, edge.previous_error = integrate(edge.error,dt) 

        x_i_dot += edge.k_p1*edge.error # proportional control 
        x_i_dot += k_i1*edge.error_int # integral control 
        edge.error_history.append(LA.norm(v_ji) - edge.weight) # append edge errors to plot simulation results

        # PI angular control
        if edge.leader: # edge.to_vertex.id == edge.leader 
            phi_ij = edge.to_vertex.current_heading - np.arctan2(-v_ji[1],-v_ji[0])  
            phi_ij = angle_wrap(phi_ij)
            edge.angular_error = angle_wrap (edge.set_angle - phi_ij) # angular error to enter into feedback loop
            e_t = np.array([-v_ji[1], v_ji[0]])/LA.norm([-v_ji[1], v_ji[0]]) # tangential basis vector 

            try:
                edge.angular_error_int, edge.previous_angular_error = integrate(edge.angular_error,dt,edge.previous_angular_error,edge.angular_error_int) 
            except:
                edge.angular_error_int, edge.previous_angular_error = integrate(edge.angular_error,dt) 

            tangential_velocity = edge.k_p2*edge.angular_error*e_t
            tangential_velocity += edge.k_i2*edge.angular_error_int*e_t                   
            x_i_dot += tangential_velocity 
            edge.angular_error_history.append(edge.angular_error)
        
        # open-loop control
        if edge.leader:
            x_i_dot += edge.to_vertex.vel
            if (edge.to_vertex.w >= 0):
                x_i_dot += -LA.norm(edge.to_vertex.w*v_ji)*e_t
            else:
                x_i_dot += LA.norm(edge.to_vertex.w*v_ji)*e_t

        # Own-axis angular control (P-control)
        if edge.leader:
            uav.heading_setpoint = angle_wrap (edge.to_vertex.current_heading)
            heading_error = angle_wrap (uav.heading_setpoint - uav.current_heading)
            k_p = 4
            uav.w = k_p*heading_error 

    return x_i_dot



def leader_damping (uav, x_i_dot, dt):
    """ damps sudden acceleration of leader drone """
    #TODO
    return x_i_dot


def leader_control (uav, formation, dt, t):
    """ feedback control for leader drones """

    # set velocity of leader drone  
    x_i_dot = formation.leader_vel(t) 
    x_i_dot = leader_damping (uav, x_i_dot, dt)

    # set angular velocity of leader drone manurally OR to match velocity direction using simple P controller
    uav.heading_setpoint = np.arctan2(x_i_dot[1],x_i_dot[0]) 
    uav.heading_setpoint = angle_wrap (uav.heading_setpoint)
    heading_error = angle_wrap (uav.heading_setpoint - uav.current_heading)
    if formation.leader_w != None:
        uav.w = formation.leader_w(t)
        uav.w_ICR = uav.w
    elif LA.norm(x_i_dot) == 0:
        uav.w = 0
    else:
        k_p = 4
        uav.w = k_p*heading_error 

    return x_i_dot


def control_iteration (formation, dt, t):
    """ performs control loop simulation for given initial conditions 
    and control input. """

    # update state machine
    if formation.state_machine.state == "MOVING" and LA.norm(formation.leader_vel(t)) == 0:
        formation.state_machine.STOP()
    elif formation.state_machine.state == "RENDEZVOUS" and LA.norm(formation.leader_vel(t)) != 0 :
        formation.state_machine.MOVE()
    
    # update contorl input for each uav (vertex in formation graph)
    # iteration for simulation purposes. Each inidividual UAV will be completing this 
    # section of code simultaneously to compute tracjetories 
    for uav in formation.vertices:

        if not uav.leader:
            # execute control for follower uavs
            x_i_dot = follower_control (uav, formation, dt)

        else:
            # set velocity of leader drone 
            x_i_dot = leader_control (uav, formation, dt, t)

        # set uav velocity
        uav.vel = x_i_dot 

        # clamp command velocity 
        if LA.norm(uav.vel) > CLAMP_VELOCITY:
            uav.vel = CLAMP_VELOCITY*uav.vel / LA.norm(uav.vel)

        # simulation results 
        uav.vel_history.append(uav.vel)

    return formation


def integrate(error, dt, previous_error=None, error_integral=None):
    if previous_error is None:
        previous_error = [0.0] * error.size
    
    if error_integral is None:
        error_integral = [0.0] * error.size   
 
    error_integral += (dt*(previous_error + error) / 2.0)
    previous_error = error.copy()

    return error_integral, previous_error



