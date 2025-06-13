import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import plotly.graph_objs as go
import plotly.io as pio


def visualize_swarm_2D(vertices, dt, xlim=None, ylim=None):
    """ code for animation to display flight simulation """

    fig = plt.figure(figsize=(8, 6)) # Set up the animation

    # Define the function to update the plot at each animation frame
    def update(frame):
        plt.clf()

        # plot the updated position of each vertex
        for vertex in vertices:
            x = vertex.path_x[frame]
            y = vertex.path_y[frame]
            vx = vertex.vel_history[frame][0]
            vy = vertex.vel_history[frame][1]

            # plot uavs (vertices) 
            plt.plot(
                x, y, "o", markersize=8, label=f"UAV {vertex.id}"
            )

            # plot velocity vectors 
            plt.quiver(
                x, y, vx, vy, angles='xy', scale_units='xy', scale=2, color='blue', alpha=0.8,
                width=0.003, headwidth=8, headlength=10, linewidth=0.5
            )
            
            # plot heading vectors
            heading_x = np.cos(vertex.heading_history[frame])
            heading_y = np.sin(vertex.heading_history[frame])
            plt.quiver(
                x, y, heading_x, heading_y, angles='xy', scale_units='xy', scale=0.5, color='red', alpha=0.8,
                width=0.003, headwidth=8, headlength=10, linewidth=0.5
            )

            # add lines between connected uavs (vertices)
            for edge in vertex.edges:
                from_x, from_y = (
                    edge.from_vertex.path_x[frame],
                    edge.from_vertex.path_y[frame],
                )
                to_x, to_y = edge.to_vertex.path_x[frame], edge.to_vertex.path_y[frame]
                plt.plot([from_x, to_x], [from_y, to_y], "-k", linewidth=0.25)

        # Set the plot limits to include all vertices, plot axis change dynamically
        if xlim == None or ylim == None:
            plt.xlim(
                min([v.path_x[frame] for v in vertices]) - 5,
                max([v.path_x[frame] for v in vertices]) + 5,
            )
            plt.ylim(
                min([v.path_y[frame] for v in vertices]) - 5,
                max([v.path_y[frame] for v in vertices]) + 5,
            )
        else:
            # Set the plot limits to a fixed width and height 
            plt.xlim(xlim[0],xlim[1])
            plt.ylim(ylim[0],ylim[1])

        # Add text box showing the time and title plot
        ax = plt.gca()
        ax.annotate(f"Time: {frame*dt:.1f}s", xy=(0.05, 0.95), xycoords="axes fraction")
        plt.legend(loc="upper right", prop={"size": 8})
        plt.grid(True)
        plt.title("UAV Formation Control")

    ani = FuncAnimation(
        fig, update, frames=len(vertices[0].path_x), interval=50, repeat=True
    )

    plt.show()


def visualize_swarm_3D(vertices, dt):
    # Set up the animation
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")

    # Define the function to update the plot at each animation frame
    def update(frame):
        # Clear the previous plot
        ax.clear()

        # Plot the updated position of each vertex
        for vertex in vertices:
            ax.scatter(
                vertex.path_x[frame],
                vertex.path_y[frame],
                vertex.path_z[0],
                c=f"C{vertex.id}",
                marker="o",
                s=30,
                label=f"UAV {vertex.id}",
            )

            # Add lines between connected vertices
            for edge in vertex.edges:
                from_x, from_y, from_z = (
                    edge.from_vertex.path_x[frame],
                    edge.from_vertex.path_y[frame],
                    edge.from_vertex.path_z[0],
                )
                to_x, to_y, to_z = (
                    edge.to_vertex.path_x[frame],
                    edge.to_vertex.path_y[frame],
                    edge.to_vertex.path_z[0],
                )
                ax.plot(
                    [from_x, to_x], [from_y, to_y], [from_z, to_z], "-k", linewidth=0.5
                )

        # Set the plot limits to include all vertices, plot axis change dynamically
        ax.set_xlim3d(
            min([v.path_x[frame] for v in vertices]) - 5,
            max([v.path_x[frame] for v in vertices]) + 5,
        )
        ax.set_ylim3d(
            min([v.path_y[frame] for v in vertices]) - 5,
            max([v.path_y[frame] for v in vertices]) + 5,
        )
        ax.set_zlim3d(
            min([v.path_z[0] for v in vertices]) - 5,
            max([v.path_z[0] for v in vertices]) + 5,
        )

        # Add text box showing the time and title plot
        ax.text2D(0.05, 0.95, f"Time: {frame*dt:.1f}s", transform=ax.transAxes)
        ax.set_title("UAV Drone Formation")

    ani = FuncAnimation(
        fig, update, frames=len(vertices[0].path_x), interval=50, repeat=True
    )
    plt.show()


def plot_error(formation, T, dt):
    """plots results of simulation, including:
    - linear error for each edge in the drone formation
    - angular error for each edge in the drone formation 
    """

    time = np.arange(0, T, dt)
    edges = formation.get_unique_edges ()

    # create plotly figure
    fig1 = go.Figure()
    fig2 = go.Figure()

    # add traces to the figure
    for edge in edges:
        fig1.add_trace(go.Scatter(x=time, y=edge.error_history, mode="lines", name=f"{edge.from_vertex.id}{edge.to_vertex.id}"))
    for edge in formation.edges:
        fig2.add_trace(go.Scatter(x=time, y=edge.angular_error_history, mode="lines", name=f"{edge.from_vertex.id}{edge.to_vertex.id}"))
    # update layout
    fig1.update_layout(
        title="Edge Errors", xaxis_title="Time [s]", yaxis_title="Errors [m]"
    )
    fig2.update_layout(
        title="Edge Angular Errors (rad)", xaxis_title="Time [s]", yaxis_title="Errors [radc]"
    )
    fig1.show()
    fig2.show()



def plot_command_vel(vertices, T, dt):
    """plots results of simulation, including:
    - command velocity for each drone
    """

    # Plot command velocities
    time = np.arange(0, T, dt)
    time_steps = int(T / dt)
    command_velocities = {vertex.id: [LA.norm(vec) for vec in vertex.vel_history] for vertex in vertices}

    for i in range(time_steps):
        for vertex in vertices:
            command_velocities[vertex.id].append(np.linalg.norm(vertex.vel_history))

        # Update vertex velocities for the next time step (example: randomly generated)
        for vertex in vertices:
            vertex.vel_history = np.random.uniform(-1, 1, size=2)

    # Create Plotly figure
    fig = go.Figure()

    # Add traces to the figure
    for vertex in vertices:
        fig.add_trace(
            go.Scatter(
                x=time,
                y=command_velocities[vertex.id],
                mode="lines",
                name=f"UAV {vertex.id}",
            )
        )

    # Update layout
    fig.update_layout(
        title="Command Velocities", xaxis_title="Time [s]", yaxis_title="Velocity [m/s]"
    )
    fig.show()


def plot_paths_2D(vertices):
    # Define traces for each drone path
    traces = []
    for vertex in vertices:
        trace = go.Scatter(
            x=vertex.path_x, y=vertex.path_y, mode="lines", name=f"UAV {vertex.id}"
        )
        traces.append(trace)

    # Define the layout of the plot
    layout = go.Layout(
        title="UAV Drone Paths",
        xaxis=dict(title="X Position [m]"),
        yaxis=dict(title="Y Position [m]"),
    )

    # Create the figure and plot the traces
    fig = go.Figure(data=traces, layout=layout)
    fig.show()
