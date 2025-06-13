""" This file plots data contained in the bagfile which is post-processed in analytics.py """

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

def plot_formation_errors (errors, type="linear"):
    """ plots edge errors of the formation during flight test """

    for topic in errors: 
        constraint_name = topic.split('/')[-1]
        plt.plot(errors[topic][0], errors[topic][1], label=constraint_name)
    
    plt.legend(fontsize=14)
    plt.title(f"UAV {type} errors")
    plt.xlabel('Time', fontsize=14)  # Increase font size for x-axis label
    if type != "angular":
        plt.ylabel('Error (m)', fontsize=14)  # Increase font size for y-axis label
    else:
        plt.ylabel('Error (deg)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers


def plot_crash_avoidance (errors, type="linear"):
    """ plots edge errors of the formation during flight test """

    for topic in errors: 
        constraint_name = topic.split('/')[-1]
        plt.plot(errors[topic][0], errors[topic][1], label=constraint_name)
    
    plt.legend(fontsize=14)
    plt.title(f"Crash Avoidance errors")
    plt.xlabel('Time', fontsize=14)  # Increase font size for x-axis label
    plt.ylabel('Absolute velocity (m/s)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers


def plot_uav_velocity (vels, type):
    """ plots the uav command velocities over time to valdate formation 
        control system """

    for key, value in vels.items():
        plt.plot(value[0], value[1], label=key)

    plt.title(f"UAV {type} velocities")
    plt.legend(fontsize=14)
    plt.xlabel('Time', fontsize=14)  # Increase font size for x-axis label
    plt.ylabel('Absolute velocity (m/s)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers



def plot_leader_angular_velocity (cmd_omega, measuerd_omega):
    """ plots the uav command velocities over time to valdate formation 
        control system """

    plt.plot(cmd_omega[0], cmd_omega[1], label="CMD")
    plt.plot(measuerd_omega[0], measuerd_omega[1], label="Measured")
    plt.title(f"Leader angular velocity")
    plt.legend(fontsize=14)
    plt.xlabel('Time', fontsize=14)  # Increase font size for x-axis label
    plt.ylabel('Absolute velocity (rad/s)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers


def plot_leader_acc (acc):
    """ plots leader drone absolute acceleration, tangential and centripetal 
        acceleration to help with acceleration based gain scheduling """

    plt.plot(acc[0], acc[1], label="X")
    plt.plot(acc[0], acc[2], label="Y")
    plt.title(f"Leader acceleration")
    plt.legend(fontsize=14)
    plt.xlabel('Time', fontsize=14)  # Increase font size for x-axis label
    plt.ylabel('Acc (m/s^2)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers


def plot_uav_paths (path):
    """ plots uav paths over course of flight test """

    for key, value in path.items():
        plt.plot(value[1], value[2], label=key)

    plt.title(f"UAV path")
    plt.legend(fontsize=14)
    plt.xlabel('x (m)', fontsize=14)  # Increase font size for x-axis label
    plt.ylabel('y (m)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers


def plot_uav_headings (headings):
    """ plots the heading angle of each uav over the flight """

    for key, value in headings.items():
        plt.plot(value[0], value[1], label=key)

    plt.title(f"UAV heading angle")
    plt.legend(fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)  # Increase font size for x-axis label
    plt.ylabel('Angle (deg)', fontsize=14)  # Increase font size for y-axis label
    plt.tick_params(axis='both', labelsize=12)  # Increase font size for axis numbers


def plot_state_changes(state_changes):
    """ plots a vertical line for each state change""" 
    for state_change in state_changes: 
        new_state = state_change[0]
        if new_state == 'LAUNCH': 
            color = 'blue'
        elif new_state == 'FORMATION_GOOD': 
            color = 'green'
        elif new_state == 'PAUSE': 
            color = 'yellow'
        elif new_state == 'FINISH': 
            color = 'red'
        else: 
            color = 'black'
        
        plt.axvline(x=state_change[1], color=color, ls='--', lw=1)


def visualize_flight_2D(uav_data, xlim=None, ylim=None):
    """ code for animation to display flight simulation """

    fig = plt.figure(figsize=(8, 6))  # Set up the animation

    # Extract time data from the first UAV (assuming all UAVs have the same time data)
    time_data = uav_data[next(iter(uav_data.keys()))][0]
    num_frames = len(time_data)

    # Synchronize waypoints in time by linear interpolation
    synchronized_uav_data = {}
    for uav_id, data in uav_data.items():
        synchronized_x = np.interp(time_data, data[0], data[1])
        synchronized_y = np.interp(time_data, data[0], data[2])
        synchronized_uav_data[uav_id] = (time_data, synchronized_x, synchronized_y)

    # Store trace points for each UAV
    trace_points = {uav_id: {"x": [], "y": []} for uav_id in synchronized_uav_data}
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']


    # Define the function to update the plot at each animation frame
    def update(frame):
        plt.clf()

        # Plot the trace points (previous path) for each UAV
        for i, (uav_id, data) in enumerate(synchronized_uav_data.items()):
            trace_x = trace_points[uav_id]["x"]
            trace_y = trace_points[uav_id]["y"]
            trace_x.append(data[1][frame])
            trace_y.append(data[2][frame])

            # Plot trace points as small transparent dots
            plt.plot(
                trace_x, trace_y, ".", markersize=1, color=colors[i], alpha=0.5
            )

        # Plot the updated position of each UAV
        for i, (uav_id, data) in enumerate(synchronized_uav_data.items()):
            time = data[0]
            x = data[1]
            y = data[2]

            # Plot UAVs
            plt.plot(
                x[frame], y[frame], "o", markersize=8, color=colors[i], label=f"UAV {uav_id}"
            )

        # Set the plot limits to include all UAVs
        if xlim is None or ylim is None:
            x_min = min([min(data[1]) for data in synchronized_uav_data.values()]) - 5
            x_max = max([max(data[1]) for data in synchronized_uav_data.values()]) + 5
            y_min = min([min(data[2]) for data in synchronized_uav_data.values()]) - 5
            y_max = max([max(data[2]) for data in synchronized_uav_data.values()]) + 5
            plt.xlim(x_min, x_max)
            plt.ylim(y_min, y_max)
        else:
            plt.xlim(xlim[0], xlim[1])
            plt.ylim(ylim[0], ylim[1])

        # Display the current time for each frame
        ax = plt.gca()
        ax.annotate(f"Time: {time[frame]:.1f}s", xy=(0.05, 0.95), xycoords="axes fraction")
        plt.legend(loc="upper right", prop={"size": 8})
        plt.grid(True)
        plt.title("UAV Formation Control")
        plt.axis('equal')

    # Reduce the interval to speed up the animation (e.g., 20ms per frame)
    ani = FuncAnimation(
        fig, update, frames=num_frames, interval=1, repeat=True
    )

    plt.show()
