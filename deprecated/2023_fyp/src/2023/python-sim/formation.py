import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from FSM import MyStateMachine


class Formation:
    """ object reperesents UAV formation and contains all flight data to be used 
    by control system """

    def __init__(self, formation_data, initial_disp, leader_id, leader_table, leader_vel, leader_w=None):
        self.vertices = []
        self.state_machine = MyStateMachine()
        self.leader_vel = leader_vel
        self.leader_w = leader_w
        adj_matrix = formation_data[0]
        self.size =len(adj_matrix) # size of formation

        # list of all edge objects (inlcude repeated edges) excluding edges where from_vertex
        # is the leader vertex however (for simulation purposes)
        self.edges = [] 

        # append vertices (uavs) to graph object
        for uav_id in range(1,len(adj_matrix)+1):
            if uav_id == int(leader_id):
                vertex = LeaderVertex(uav_id, initial_disp[f"{uav_id}"])
            else:
                vertex = Vertex(uav_id, initial_disp[f"{uav_id}"])
            self.vertices.append(vertex)

        # append edges (communication links) to graph object
        for row in range(len(adj_matrix)):
            for col in range(len(adj_matrix)):
                if adj_matrix[row][col] != 0:
                    weight = adj_matrix[row][col]
                    set_angle = formation_data[1][row][col]
                    local_leader = True if leader_table[row][col] == 1 else False
                    k_p1 = formation_data[2][row][col]
                    k_i1 = formation_data[3][row][col]
                    k_p2 = formation_data[4][row][col]
                    k_i2 = formation_data[5][row][col]
                    edge = self.add_edge(row+1, col+1, weight, set_angle, k_p1, k_i1, k_p2, k_i2, local_leader)
                    if row+1 != int(leader_id):
                        self.edges.append(edge) 

    def add_vertex(self, vertex):
        self.vertices.append(vertex)

    def add_edge(self, from_vertex_id, to_vertex_id, weight, set_angle, k_p1, k_i1, k_p2, k_i2, local_leader):
        from_vertex = self.get_vertex(from_vertex_id)
        to_vertex = self.get_vertex(to_vertex_id)
        edge = Edge(from_vertex, to_vertex, weight, set_angle, k_p1, k_i1, k_p2, k_i2, local_leader)
        from_vertex.add_edge(edge)

        return edge

    def get_vertex(self, vertex_id):
        for vertex in self.vertices:
            if vertex.id == vertex_id:
                return vertex
        return None
    
    def get_unique_edges(self):
        unique_edges = list(set(self.edges)) 
        return unique_edges

    def set_leader(self, vertex_id):
        uav_leader = self.get_vertex(vertex_id)
        uav_leader.leader = True
    
    def visualize (self):
        # create a new networkx graph
        G = nx.Graph()

        # add all vertices as nodes to the graph
        for vertex in self.vertices:
            G.add_node(vertex.id)

        # add all edges as edges to the graph
        for vertex in self.vertices:
            for edge in vertex.edges:
                G.add_edge(edge.from_vertex.id, edge.to_vertex.id, weight=edge.weight)

        # define positions for each node
        pos = nx.spring_layout(G, weight='weight', scale=2)

        # draw nodes and edges with networkx
        nx.draw_networkx_nodes(G, pos, node_color='blue', node_size=500)
        nx.draw_networkx_edges(G, pos, edge_color='gray')

        # add labels to nodes
        labels = {}
        for vertex in self.vertices:
            labels[vertex.id] = str(vertex.id)
        nx.draw_networkx_labels(G, pos, labels, font_size=16)

        # add labels to edges
        edge_labels = {}
        for (u, v, d) in G.edges(data=True):
            edge_labels[(u, v)] = str(d['weight'])
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12)

        # show the plot
        plt.title("Drone Formation Graph")
        plt.axis('off')
        plt.show()
        
    def __str__(self):
        vertex_str = ""
        for vertex in self.vertices:
            vertex_str += f"{vertex}\n"
            for edge in vertex.edges:
                vertex_str += f"  {edge}\n"
        return vertex_str


class Vertex:
    """ each UAV in the formation are vertices in the 
        formation graph """

    def __init__(self, id, initial_disp):
        self.id = id
        self.edges = []
        self.disp = initial_disp
        self.vel = np.array([0,0], dtype=np.float64)
        self.acc = np.array([0,0], dtype=np.float64)
        self.current_heading = np.pi/2
        self.leader = False
        self.w_ICR = 0 # angular velocity about the instantaneous centre of rotation
        self.w = 0 # angular velocity of leader relative to itself (rad/s)
        self.heading_setpoint = np.pi/2 # desired heading angle (rad)
        self.current_heading = np.pi/2 # current heading angle (rad)

        # measurements for simulation results
        self.path_x = [initial_disp[0]]
        self.path_y = [initial_disp[1]]
        self.path_z = [0]
        self.vel_history = [np.array([0,0])]
        self.heading_history = [np.array(0)] # measured heading angle over time
        
    def add_edge(self, edge):
        self.edges.append(edge)

    def __str__(self):
        return f"Vertex {self.id}"
    

class LeaderVertex (Vertex):
    """ Object which reperesents the leader uav (could be virtual) in
        in the formation. Class inherits Vertex class. """
    
    def __init__(self, id, initial_disp):
        super().__init__(id, initial_disp)
        self.leader = True


class Edge:
    """ formation graph edges reperesent connections in the formation between
        neighbouring drones """
    
    def __init__(self, from_vertex, to_vertex, weight, set_angle, k_p1, k_i1, k_p2, k_i2, local_leader):
        self.from_vertex = from_vertex
        self.to_vertex = to_vertex
        self.weight = weight  # desired distance between vertices
        self.set_angle = set_angle # desired angle between leader uav heading and edge in radians
        self.k_p1 = k_p1 # linear P gain
        self.k_i1 = k_i1 # linear I gain
        self.k_p2 = k_p2 # angular P gain
        self.k_i2 = k_i2 # angular I gain
        self.previous_error = np.array([0,0])
        self.error = np.array([0,0])
        self.error_derivative = np.array([0,0])
        self.error_int = np.array([0,0])
        self.error_history = [] 
        self.previous_angular_error = 0
        self.angular_error = 0
        self.angular_error_int = 0
        self.angular_error_history = []
        self.leader = local_leader # true if edge to_vertex is a leader drone of the edge from_vertex

    def __hash__(self):
        # Define a hash function for the Edge object
        # This allows it to be used in a set data structure
        return hash(frozenset((self.from_vertex, self.to_vertex)))


    def __eq__(self, other):
        # Check if the edges are the same, regardless of the order
        return (self.from_vertex == other.from_vertex and self.to_vertex == other.to_vertex) or \
               (self.from_vertex == other.to_vertex and self.to_vertex == other.from_vertex)

    def __str__(self):
        return f"Edge from {self.from_vertex} to {self.to_vertex}"




