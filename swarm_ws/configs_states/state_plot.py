import json
from graphviz import Digraph
import tkinter as tk
from tkinter import filedialog, messagebox

# Define the State class
class State:
    def __init__(self, name, off_board=False, valid=None, on_enter=None, on_exit=None):
        self.name = name
        self.off_board = off_board
        self.transitions = {}  # key: destination state, value: list of events
        self.on_enter = on_enter
        self.on_exit = on_exit

    def add_transition(self, to_state, events=None):
        if events is None:
            events = []
        self.transitions[to_state] = events

    def __repr__(self):
        return f"State(name={self.name}, transitions={self.transitions})"

def read_config(config_file):
    with open(config_file, 'r') as f:
        data = json.load(f)

    states = {}
    begin_state = None

    # First, create all state objects
    for item in data:
        state_name = item.get('state')
        off_board = item.get('off_board', False)
        valid_transitions = item.get('valid', [])
        on_enter = item.get('onEnter')
        on_exit = item.get('onExit')

        state = State(
            name=state_name,
            off_board=off_board,
            on_enter=on_enter,
            on_exit=on_exit
        )

        # Check if this is the begin state
        if item.get('begin', False):
            begin_state = state_name

        states[state_name] = state

    # Now, add transitions and validate
    for item in data:
        state_name = item.get('state')
        valid_transitions = item.get('valid', [])

        state = states[state_name]

        for transition in valid_transitions:
            # Check if events are associated with the transition
            if isinstance(transition, dict):
                dest_state = transition['state']
                events = transition.get('events', [])
            else:
                dest_state = transition
                events = []

            # Validate destination state
            if dest_state not in states:
                raise ValueError(f"Transition to undefined state '{dest_state}' from state '{state_name}'.")

            state.add_transition(dest_state, events)

    return states, begin_state

def generate_state_diagram(states, begin_state=None, output_file='state_diagram', graph_width=11, graph_height=8.5):
    dot = Digraph(comment='State Diagram', format='png')

    # Set graph size and layout
    dot.attr(size=f"{graph_width},{graph_height}!")
    dot.attr(rankdir='LR')  # Left to right layout
    dot.attr(splines='ortho')  # Orthogonal routing for cleaner paths

    # Define styles for nodes and edges
    node_styles = {
        'start': {
            'shape': 'circle',
            'style': 'filled',
            'fillcolor': '#FFD700',  # Gold
            'fontname': 'Helvetica',
            'fontsize': '12',
            'color': 'black'
        },
        'onboard': {
            'shape': 'ellipse',
            'style': 'filled',
            'fillcolor': '#ADD8E6',  # Light blue
            'fontname': 'Helvetica',
            'fontsize': '12',
            'color': 'black'
        },
        'offboard': {
            'shape': 'doubleoctagon',
            'style': 'filled,bold',
            'fillcolor': '#FFA500',  # Orange
            'fontname': 'Helvetica',
            'fontsize': '14',
            'color': 'black',
            'penwidth': '2'
        },
        'hold_pause': {
            'shape': 'rectangle',
            'style': 'filled,rounded',
            'fillcolor': '#90EE90',  # Light green
            'fontname': 'Helvetica',
            'fontsize': '12',
            'color': 'black'
        }
    }

    edge_styles = {
        'normal': {
            'fontname': 'Helvetica',
            'fontsize': '10',
            'color': '#7DCEA0',  # Green
            'arrowsize': '0.8'
        },
        'highlight_offboard': {
            'fontname': 'Helvetica',
            'fontsize': '10',
            'color': '#FF4500',  # OrangeRed
            'arrowsize': '1.0',
            'penwidth': '2'
        },
        'highlight_hold': {
            'fontname': 'Helvetica',
            'fontsize': '10',
            'color': '#1E90FF',  # DodgerBlue
            'arrowsize': '1.0',
            'penwidth': '2'
        },
        'highlight_both': {
            'fontname': 'Helvetica',
            'fontsize': '10',
            'color': '#800080',  # Purple
            'arrowsize': '1.2',
            'penwidth': '3',
            'style': 'dashed'
        },
        'enter_section': {
            'fontname': 'Helvetica',
            'fontsize': '10',
            'color': '#000000',  # Black
            'arrowsize': '1.0',
            'penwidth': '2',
            'style': 'solid'
        }
    }

    # Create subgraphs (clusters) for different state groups
    onboard_cluster = Digraph(name='cluster_onboard')
    onboard_cluster.attr(label='Onboard States', style='filled', color='#E0E0E0')

    offboard_cluster = Digraph(name='cluster_offboard')
    offboard_cluster.attr(label='Offboard States', style='filled', color='#FFF2CC')

    hold_pause_cluster = Digraph(name='cluster_hold_pause')
    hold_pause_cluster.attr(label='Hold/Pause States', style='filled', color='#F0FFF0')

    # Add nodes to respective clusters
    for state_name, state in states.items():
        label = f"{state_name}"
        if 'pause' in state_name.lower() or 'hold' in state_name.lower():
            node_attr = node_styles['hold_pause']
            hold_pause_cluster.node(state_name, label, **node_attr)
        elif state.off_board:
            node_attr = node_styles['offboard']
            offboard_cluster.node(state_name, label, **node_attr)
        else:
            node_attr = node_styles['onboard']
            onboard_cluster.node(state_name, label, **node_attr)

    # Combine clusters
    dot.subgraph(onboard_cluster)
    dot.subgraph(offboard_cluster)
    dot.subgraph(hold_pause_cluster)

    # Add Start Node and connect to 'configure'
    dot.node('start', **node_styles['start'])
    if begin_state and begin_state in states:
        dot.edge('start', begin_state, label='', **edge_styles['normal'])

    # Add transitions
    for state_name, state in states.items():
        for dest_state, events in state.transitions.items():
            if events:
                label = ', '.join(events)
            else:
                label = ''

            # Determine transition type
            from_offboard = states[state_name].off_board
            from_hold_pause = 'pause' in state_name.lower() or 'hold' in state_name.lower()
            to_offboard = states[dest_state].off_board
            to_hold_pause = 'pause' in dest_state.lower() or 'hold' in dest_state.lower()

            # Determine edge style
            if (from_offboard and to_hold_pause) or (from_hold_pause and to_offboard):
                edge_style = edge_styles['highlight_both']
            elif from_offboard or to_offboard:
                edge_style = edge_styles['highlight_offboard']
            elif from_hold_pause or to_hold_pause:
                edge_style = edge_styles['highlight_hold']
            else:
                edge_style = edge_styles['normal']

            # Apply style attributes
            edge_attrs = {}
            for key, value in edge_style.items():
                edge_attrs[key] = value

            dot.edge(state_name, dest_state, label=label, **edge_attrs)

    # Prevent arrows from passing through cluster boundaries unless entering
    # This is handled implicitly by Graphviz's layout, but to enhance it:
    # Use port-based connections or rank constraints if needed
    # For simplicity, we rely on Graphviz's orthogonal routing and clustering

    # Reduce arrow crossings by arranging clusters and nodes logically
    # This is a complex task and is largely handled by Graphviz's layout engine
    # However, we can influence it using ranks and subgraphs

    # Save and render the diagram
    dot.render(output_file, view=True)

# GUI Application
class StateDiagramGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("State Diagram Generator")

        self.config_file = None
        self.states = None
        self.begin_state = None

        # Width and Height sliders
        self.width_var = tk.DoubleVar(value=11)
        self.height_var = tk.DoubleVar(value=8.5)

        self.create_widgets()

    def create_widgets(self):
        # Configuration File Selection
        tk.Label(self.root, text="Configuration File:").grid(row=0, column=0, sticky='e')
        self.config_entry = tk.Entry(self.root, width=50)
        self.config_entry.grid(row=0, column=1, padx=5, pady=5)
        tk.Button(self.root, text="Browse...", command=self.browse_config).grid(row=0, column=2, padx=5)

        # Width Slider
        tk.Label(self.root, text="Graph Width (inches):").grid(row=1, column=0, sticky='e')
        self.width_slider = tk.Scale(self.root, from_=5, to=30, orient=tk.HORIZONTAL, resolution=0.5, variable=self.width_var)
        self.width_slider.grid(row=1, column=1, columnspan=2, sticky='we', padx=5, pady=5)

        # Height Slider
        tk.Label(self.root, text="Graph Height (inches):").grid(row=2, column=0, sticky='e')
        self.height_slider = tk.Scale(self.root, from_=5, to=30, orient=tk.HORIZONTAL, resolution=0.5, variable=self.height_var)
        self.height_slider.grid(row=2, column=1, columnspan=2, sticky='we', padx=5, pady=5)

        # Generate Button
        self.generate_button = tk.Button(self.root, text="Generate State Diagram", command=self.generate_diagram)
        self.generate_button.grid(row=3, column=0, columnspan=3, pady=10)

    def browse_config(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir='.'
        )
        if file_path:
            self.config_file = file_path
            self.config_entry.delete(0, tk.END)
            self.config_entry.insert(0, file_path)

    def generate_diagram(self):
        if not self.config_entry.get():
            messagebox.showerror("Error", "Please select a configuration file.")
            return

        try:
            self.states, self.begin_state = read_config(self.config_entry.get())
            width = self.width_var.get()
            height = self.height_var.get()
            output_file = 'state_diagram'
            generate_state_diagram(
                self.states,
                self.begin_state,
                output_file=output_file,
                graph_width=width,
                graph_height=height
            )
            messagebox.showinfo("Success", f"State diagram generated successfully as '{output_file}.png'!")
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred:\n{e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = StateDiagramGUI(root)
    root.mainloop()