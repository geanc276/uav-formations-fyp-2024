"""
    File containing the constraints which have been implemented for the drones. 

    Each constraint implements the following methods: 
        - __init__ with whatever parameters it needs
        - from_dict a class method that loads the constraint from a dictionary
        - generate_error a method that generates the error based on a dict of 
        neighbour positions (in the form {str: DronePos}) and also the current drone position
"""
from simple_pid import PID

from std_msgs.msg import Float64MultiArray

import rospy
import warnings
import numpy as np
from dronepos import DronePos
from gps import GPSCoord

class Constraint(): 
    def __init__(self): 
        self.logger = None


    """
    Log a correction from a constraint to the associated ROS topic
    """
    def log(self, error: float, gains: tuple, output: np.array): 
        # No provided logger
        if self.logger is None:
            rospy.loginfo(error)
            return
        
        message = Float64MultiArray()
        message.data = [
            error, 
            *gains, 
            *output
        ]
        self.logger.publish(message)


    """
    Abstract method
    """
    def generate_error(self, neighbour_pos, drone_pos: DronePos): 
        raise NotImplementedError()
    

""" 
Constraint that the drone be a certain distance away from a fixed
reference point
"""
class FixedLocalConstraint(Constraint): 
    def __init__(self, position: str, distance: float, controller: PID): 
        self.position = position
        self.distance = distance
        self.controller = controller
        self.controller.setpoint = distance
        super(FixedLocalConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['position'],            # Drone to follow
            config['distance'],         # Set point distance 
            PID(*config['gains'])       # PID controller gains
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Compute the distance from the reference
        distance = np.sqrt((drone_pos.local.position.x - self.position[0]) ** 2 + 
                           (drone_pos.local.position.y - self.position[1]) ** 2)

        # Compute the control input
        direction = np.array([
            drone_pos.local.position.x - self.position[0],
            drone_pos.local.position.y - self.position[1],
            0,
            0, 
            0, 
            0
        ])

        if (np.linalg.norm(direction) == 0):
            output = np.array([0,0,0,0,0,0])
        else:
            direction = direction / np.linalg.norm(direction)   # Normalise the direction vector
            output = direction * self.controller(distance)

        self.log(self.distance - distance, self.controller.components, output)

        return output


""" 
Constraint that the drone be a certain distance away from another
drone
"""
class DistanceConstraint(Constraint): 
    def __init__(self, other: str, distance: float, gains1, gains2, controller: PID): 
        self.other = other
        self.radius = distance
        self.gains1 = gains1
        self.gains2 = gains2
        self.controller = controller
        self.controller.setpoint = distance
        super(DistanceConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'],            # Drone to follow
            config['distance'],         # Set point distance 
            config['gains1'],
            config['gains2'],
            PID(*config['gains1'])       # PID controller gains
        )

    def update_gains(self, num):
        if num == 1:
            self.controller.Kp, self.controller.Ki, self.controller.Kd  = [self.gains1[0], self.gains1[1], self.gains1[2]]
        else:
            self.controller.Kp, self.controller.Ki, self.controller.Kd  = [self.gains2[0], self.gains2[1], self.gains2[2]]

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Get the position of the other drone 
        other_drone_pos: GPSCoord = neighbour_pos[self.other].global_pos

        # Compute the current distance between the two drones
        distance = drone_pos.global_pos.distance(other_drone_pos)

        # Compute the control input
        direction = np.array([
            other_drone_pos.x_distance(drone_pos.global_pos), 
            other_drone_pos.y_distance(drone_pos.global_pos), 
            0,
            0, 
            0, 
            0
        ])
        direction = direction / np.linalg.norm(direction)   # Normalise the direction vector
        output = direction * self.controller(distance)
        self.log(self.radius - distance, self.controller.components, output)
        return output


""" 
Constraint that ensures two drones do not crash, by providing a highly nonlinear
correction as the drones get close to a certain point. The curve is parameterised by 
these parameters: 
d_m - minimum distance
d_M - cutoff distance
c - constant
\left\{x<d_{m}:0,x<d_{M}:K\left(\frac{\left(d_{M}-d_{m}\right)^{2}}{\left(x-d_{m}\right)^{2}}\right)+c-K,x>d_{M}:0\right\}
"""
class CrashAvoidanceConstraint(Constraint): 
    def __init__(self, other: str, dmin: float, dmax: float, K: float, c: float): 
        self.other = other
        self.dmin = dmin
        self.dmax = dmax
        self.K = K
        self.c = c
        super(CrashAvoidanceConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'],
            config['dmin'],            # Drone to follow
            config['dmax'],         # Set point distance 
            config['K'],
            config['c']
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Get the position of the other drone 
        other_drone_pos: GPSCoord = neighbour_pos[self.other].global_pos

        # Compute the current distance between the two drones
        distance = drone_pos.global_pos.distance(other_drone_pos)

        # Compute the control input
        direction = np.array([
            other_drone_pos.x_distance(drone_pos.global_pos), 
            other_drone_pos.y_distance(drone_pos.global_pos), 
            0,
            0, 
            0, 
            0
        ])
        direction = direction / np.linalg.norm(direction)   # Normalise the direction vector

        if distance < self.dmin or distance > self.dmax: 
            output_mag = 0
        elif distance < self.dmax: 
            output_mag = self.K * ((self.dmax - self.dmin) / (distance - self.dmin)) ** 2 + self.c - self.K

        output = direction * output_mag

        self.log(output_mag, [output_mag, 0, 0], output)
        return output


""" 
Constraint that the drone be a certain distance away from another
drone in 3D space
"""
class Distance3dConstraint(Constraint): 
    def __init__(self, other: str, distance: float, controller: PID): 
        self.other = other
        self.radius = distance
        self.controller = controller
        self.controller.setpoint = distance
        super(Distance3dConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'],            # Drone to follow
            config['distance'],         # Set point distance 
            PID(*config['gains'])       # PID controller gains
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Get the position of the other drone 
        other_drone_pos: GPSCoord = neighbour_pos[self.other].global_pos

        # Compute the current distance between the two drones
        distance = drone_pos.global_pos.distance3d(other_drone_pos)

        # Compute the control input
        direction = np.array([
            other_drone_pos.x_distance(drone_pos.global_pos), 
            other_drone_pos.y_distance(drone_pos.global_pos), 
            other_drone_pos.z_distance(drone_pos.global_pos), 
            0, 
            0, 
            0
        ])
        direction = direction / np.linalg.norm(direction)   # Normalise the direction vector
        output = direction * self.controller(distance)
        self.log(self.radius - distance, self.controller.components, output)
        return output



"""
Constraint representing a simple fixed-altitude constraint
"""
class AltitudeConstraint(Constraint): 
    def __init__(self, alt: float, gains1, gains2, controller: PID): 
        self.alt = alt
        self.gains1 = gains1
        self.gains2 = gains2
        self.controller = controller
        self.controller.setpoint = alt
        super(AltitudeConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['alt'],            # Drone to follow
            config['gains1'],
            config['gains2'],
            PID(*config['gains1'])       # PID controller gains
        )

    def update_gains(self, num):
        if num == 1:
            self.controller.Kp, self.controller.Ki, self.controller.Kd  = [self.gains1[0], self.gains1[1], self.gains1[2]]
        else:
            self.controller.Kp, self.controller.Ki, self.controller.Kd  = [self.gains2[0], self.gains2[1], self.gains2[2]]

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Get the current (local) altitude
        altitude = drone_pos.local.position.z
        direction = np.array([0, 0, 1, 0, 0, 0])
        output = direction * self.controller(altitude)
        self.log(altitude - self.alt, self.controller.components, output)
        return output



"""
Constraint representing the fact that certain two drones should be at the same altitude
"""
class SameAltitudeConstraint(Constraint): 
    def __init__(self, other: str, controller: PID): 
        self.other = other
        self.controller = controller
        self.controller.setpoint = 0
        super(SameAltitudeConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'],            # Drone to follow
            PID(*config['gains'])       # PID controller gains
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Get the difference between our altitude and the other altitude
        alt_difference = neighbour_pos[self.other].global_pos.z_distance(drone_pos.global_pos)
        direction = np.array([0, 0, 1, 0, 0, 0])
        output = direction * self.controller(alt_difference)
        self.log(alt_difference, self.controller.components, output)
        return output



"""
Constraint representing the fact that a drone should be in between two other drones, 
and those three drones should be collinear. 
"""
class CollinearityConstraint(Constraint): 
    def __init__(self, other, distances, controller: PID): 
        self.other1 = other[0]      # First drone name
        self.other2 = other[1]      # Second drone name
        self.d1 = distances[0]      # First drone distance
        self.d2 = distances[1]      # Second drone distance
        self.controller = controller
        self.controller.setpoint = 0
        super(CollinearityConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'],            # Drone to follow, 
            config['distances'],        # Distances
            PID(*config['gains'])       # PID controller gains
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Get the point that it should be at (approximation for small distances)
        drone1_pos: GPSCoord = neighbour_pos[self.other1].global_pos
        drone2_pos: GPSCoord = neighbour_pos[self.other2].global_pos
        set_point = GPSCoord(
            (drone1_pos.lat * self.d2 + drone2_pos.lat * self.d1) / (self.d1 + self.d2), 
            (drone1_pos.long * self.d2 + drone2_pos.long * self.d1) / (self.d1 + self.d2), 
            0
        )

        # Get the direction to the set point
        direction = np.array([
            set_point.x_distance(drone_pos.global_pos), 
            set_point.y_distance(drone_pos.global_pos), 
            0, 
            0, 
            0, 
            0
        ])
        direction = direction / np.linalg.norm(direction)

        # Get the correction
        distance = drone_pos.global_pos.distance(set_point)
        output = direction * self.controller(distance)
        self.log(distance, self.controller.components, output)
        return output



def clamp_heading(heading): 
    """
    Clamp a heading to a value between -180 and +180
    """
    if heading < -180.0: 
        return clamp_heading(heading + 360.0)
    elif heading > 180.0: 
        return clamp_heading(heading - 360.0)
    return heading


def convert_heading (heading):
    """"
        convertes drone headings being calculated as positive clockwise to heading angles
        being calculated as positive anti-clockwise
    """

    heading = clamp_heading (heading)
    if heading > 0:
        heading = 180 - heading
    else:
        heading = -heading-180

    return heading


"""
Constraint on a drone's heading
"""

class HeadingConstraint(Constraint): 
    def __init__(self, heading, controller: PID): 
        self.heading = heading
        self.controller = controller
        self.controller.setpoint = 0
        super(HeadingConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['heading'], 
            PID(*config['gains'])       # PID controller gains
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Compute the heading error
        cur_heading = drone_pos.heading
        heading_error = clamp_heading(self.heading - cur_heading)

        direction = np.array([0, 0, 0, 0, 0, 1.0])
        # Get the correction
        output = direction * self.controller(heading_error)
        self.log(heading_error, self.controller.components, output)
        return output
    

"""
Constraint that forces two drones to have the same heading
"""
class SameHeadingConstraint(Constraint): 
    def __init__(self, other, controller: PID): 
        self.other = other
        self.controller = controller
        self.controller.setpoint = 0
        super(SameHeadingConstraint, self).__init__()
    
    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'], 
            PID(*config['gains'])
        )

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Compute the heading error
        cur_heading = drone_pos.heading
        other_heading = neighbour_pos[self.other].heading
        heading_error = clamp_heading(other_heading - cur_heading)

        direction = np.array([0, 0, 0, 0, 0, 1.0])
        # Get the correction
        output = direction * self.controller(heading_error)
        self.log(heading_error, self.controller.components, output)
        return output



"""
Constraint which causes a certain drone to be at a certain angle relative to
the heading of another drone, e.g. setting B, 90 degrees, A gives 
                A -----> 
                |
                |
                B
"""
class AngularConstraint(Constraint): 
    def __init__(self, other, angle, gains1, gains2, controller: PID): 
        self.other = other
        self.angle = angle
        self.gains1 = gains1
        self.gains2 = gains2
        self.controller = controller
        self.controller.setpoint = 0
        super(AngularConstraint, self).__init__()
    
    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            config['other'], 
            config['angle'], 
            config['gains1'],
            config['gains2'],
            PID(*config['gains1'])
        )
    
    def update_gains(self, num):
        if num == 1:
            self.controller.Kp, self.controller.Ki, self.controller.Kd  = [self.gains1[0], self.gains1[1], self.gains1[2]]
        else:
            self.controller.Kp, self.controller.Ki, self.controller.Kd  = [self.gains2[0], self.gains2[1], self.gains2[2]]
        

    def generate_error(self, neighbour_pos, drone_pos: DronePos) -> np.array: 
        # Compute the error in the relative bearing between the two drones
        other_pos: DronePos = neighbour_pos[self.other]

        # recaluclate drone heading as convention does not much coordinate frame (individual drones measure anticlockwise -ve)
        leader_heading = convert_heading (other_pos.heading)
        edge_angle = np.rad2deg (np.arctan2 (other_pos.global_pos.y_distance(drone_pos.global_pos), other_pos.global_pos.x_distance(drone_pos.global_pos)))
        setpoint = clamp_heading(leader_heading + self.angle) # setpoint global yaw space  
        error = clamp_heading(setpoint - edge_angle) 

        # Calculate and normalise the direction
        direction = np.array([
            -other_pos.global_pos.y_distance(drone_pos.global_pos), 
            other_pos.global_pos.x_distance(drone_pos.global_pos), 
            0, 
            0, 
            0, 
            0
        ])
        direction = direction / np.linalg.norm(direction)

        # Get the correction
        output = -direction * self.controller(error)
        self.log(error, self.controller.components, output)
        return output
    
class HoldConstraint(Constraint):
    def __init__(self, controller: PID): 
        self.controller = controller
        self.controller.setpoint = 0
        super(HoldConstraint, self).__init__()

    @classmethod
    def from_dict(cls, config: dict): 
        return cls(
            PID(*config['gains'])       # PID controller gains
        )

    def generate_error(self, hold_pos: GPSCoord, drone_pos: DronePos) -> np.array: 
        # Get the hold position of the drone 
        other_drone_pos: GPSCoord = hold_pos

        # Compute the current distance between the two drones
        distance = drone_pos.global_pos.distance3d(other_drone_pos)

        # Compute the control input
        direction = np.array([
            other_drone_pos.x_distance(drone_pos.global_pos), 
            other_drone_pos.y_distance(drone_pos.global_pos), 
            other_drone_pos.z_distance(drone_pos.global_pos), 
            0, 
            0, 
            0
        ])
        if np.linalg.norm(direction) != 0:
            direction = direction / np.linalg.norm(direction)   # Normalise the direction vector
            output = direction * self.controller(distance)
        else:
            output = np.zeros((6,))

        self.log(distance, self.controller.components, output)
        return output

type_name = {
    'fixed-local': FixedLocalConstraint, 
    'distance': DistanceConstraint, 
    'crash-avoidance': CrashAvoidanceConstraint, 
    'distance3': Distance3dConstraint, 
    'altitude': AltitudeConstraint, 
    'same-alt': SameAltitudeConstraint, 
    'collinear': CollinearityConstraint, 
    'heading': HeadingConstraint, 
    'same-hdg': SameHeadingConstraint, 
    'angular': AngularConstraint,
    'hold': HoldConstraint
}


"""
Function to parse a constraint
"""
def parse_constraint(config: dict) -> Constraint: 
    return type_name[config['type']].from_dict(config)


