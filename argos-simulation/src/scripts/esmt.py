
import os.path
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
import subprocess

GEOSTEINER_DIR_NAME = 'geosteiner-5.3/'


class ESMT:
    """
    Finds a an optimal solution for the Euclidean Steiner Minimum Tree problem.
    Additional points are then added to satisfy communication range constraints and teams that exist along the network
    """

    def __init__(self, pos, radius) -> None:
        self.pos = pos
        self.R = radius
        
        self.G = nx.Graph()
        self.added_points = 0
        self.length = 0


    def solve(self, mode='release'):

        if mode == 'plot':
            fig = plt.figure()
            ax = fig.gca()

        geosteiner_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), GEOSTEINER_DIR_NAME)
        # print(geosteiner_path)

        ### Step 1
        # Add terminals to the graph
        # Prepare coordinates then in text format

        self.G = nx.Graph()
        self.G.add_nodes_from(self.pos)
        # print(self.G)


        ### Step 2
        # Call the GeoSteiner package to solve the Euclidean Steiner Minimum Tree
        # Parse output of Steiner points
        # Add Steiner points to the graph

        # Store terminals to temp file
        txt_content = '1\n' + str(len(self.pos)) + '\n'
        for val in self.pos.values():
            txt_content += str(val[0]) + ' ' + str(val[1]) + '\n'
        # print(txt_content)

        f = open(os.path.join(geosteiner_path, 'temp.txt'), 'w')
        f.write(txt_content)
        f.close()

        # Read temp file and run the GeoSteiner package
        f = open(os.path.join(geosteiner_path, 'temp.txt'))
        # proc1 = subprocess.Popen([geosteiner_path+'rand_points', '6'], stdout=subprocess.PIPE)
        proc1 = subprocess.Popen([geosteiner_path+'lib_points'], stdin=f, stdout=subprocess.PIPE)

        proc2 = subprocess.Popen([geosteiner_path+'efst'], stdin=proc1.stdout, stdout=subprocess.PIPE)
        # print(output2.stdout.decode('UTF-8'))
        proc3 = subprocess.Popen([geosteiner_path+'bb'], stdin=proc2.stdout, stdout=subprocess.PIPE)
        # print(output3.stdout.decode('UTF-8'))

        output = proc3.stdout.readlines()

        # Get all Steiner points
        s_count = 0
        for line in output:
            # Check if line contains Steiner point info
            str_line = line.decode('UTF-8')
            split_line = str_line.split('\t')
            str_point_info = split_line[0][3:5]
            if(str_point_info == '@C'):
                # print(split_line)
                s_count += 1
                s_point = f'S{s_count}'
                self.pos[s_point] = (float(split_line[1]), float(split_line[2].split('\n')[0]))

        # print(self.pos)

        # Get all edges
        for line in output:
            # Check if line contains edge info
            str_line = line.decode('UTF-8')
            split_line = str_line.split('\t')
            str_edge_info = split_line[-1].split('\n')[0]

            if(str_edge_info == 'S'):

                nodes = []

                # loop line
                    # if it is a terminal
                        # add it and continue loop
                    # else
                        # use the current and next element to add steiner point

                edge_line = split_line[1:][:-1]
                # print(edge_line)
                i = 0
                while i < len(edge_line):
                    # print(edge_line)
                    if(edge_line[i][-1] == 'T'):
                        # Terminal
                        nodes.append(int(edge_line[i][:-2]) + 1)
                    else:
                        # Steiner point
                        x = float(edge_line[i])
                        y = float(edge_line[i+1])
                        result = list(self.pos.keys())[list(self.pos.values()).index((x,y))]
                        # print(result, x, y)
                        nodes.append(result)
                        i += 1

                    i += 1

                # print(f'nodes: {nodes}')

                # TEMP: NEED TO STEINERIZE
                self.G.add_edge(nodes[0], nodes[1], length=math.dist(self.pos[nodes[0]], self.pos[nodes[1]]))

        # print(self.G.edges)
        # print(self.G.size(weight='length'))


        ### Step 3
        # For each edge with |e| > R, Steinerize the edge by adding further points to the graph

        # loop edge
            # if dist > R
                # remove the edge from the graph
                # calculate position of new points
                # Add new points to graph and pos

        temp_loop_edges = copy.deepcopy(self.G.edges)

        for pair in temp_loop_edges:
            d = self.G.edges[pair]['length']
            # print(pair, d)

            if(d > self.R):
                # print('edge too long', d)

                # Remove edge
                self.G.remove_edge(pair[0], pair[1])

                x1, y1 = self.pos[pair[0]]
                x2, y2 = self.pos[pair[1]]

                # Number of Steiner points to insert
                num_insert = math.ceil(d/self.R) - 1
                # print(num_insert)

                # Calculate the coordinates of the new Steiner points
                new_x = np.linspace(x1, x2, num=(num_insert+2))
                new_y = np.linspace(y1, y2, num=(num_insert+2))

                for i in range(num_insert):
                    s_count += 1
                    s_point = f'S{s_count}'
                    self.pos[s_point] = (new_x[i+1], new_y[i+1])
                    prev_node = f'S{s_count-1}'
                    next_node = f'S{s_count+1}'
                    if i == 0:
                        prev_node = pair[0]
                    if i == num_insert - 1:
                        next_node = pair[1]
                    self.G.add_edge(s_point, prev_node, length=math.dist(self.pos[pair[0]], self.pos[pair[1]]) / (num_insert + 1))
                    self.G.add_edge(s_point, next_node, length=math.dist(self.pos[pair[0]], self.pos[pair[1]]) / (num_insert + 1))

                    # if mode == 'debug' or mode == 'plot':
                    #     print(f'\tAdding Steiner point {s_point} between ({v1},{v2}) at {self.pos[s_point]}')

        # print(self.G.nodes)
        # print(self.G.edges)
        

        ### Step 4
        # If a team has a degree > 1, add that point to the graph

        # loop nodes
            # if node has degree > 1
                # add new point
                # remove all edges that belonged to node
                # add all egdes that belonged to node
                # add new edge from node to point

        temp_loop_nodes = copy.deepcopy(self.G.nodes)
        for node in temp_loop_nodes:
            if(self.G.degree[node] > 1 and str(node)[0] != 'S'):
                # print(f'node in between! {node}')

                # Add new point
                s_count += 1
                s_point = f'S{s_count}'
                self.pos[s_point] = (self.pos[node][0], self.pos[node][1])
                # print(self.pos)
                # self.G.add_node(s_point)

                neighbors = list(self.G.neighbors(node))

                # Remove all edges that belong to the node
                # print(self.G.edges(node))
                temp_edges = copy.deepcopy(self.G.edges(node))
                self.G.remove_edges_from(temp_edges)

                # Add all egdes that belonged to node
                self.G.add_node(s_point)
                for n in neighbors:
                    self.G.add_edge(s_point, n, length=math.dist(self.pos[s_point], self.pos[n]))

                # Add new edge from node to point
                self.G.add_edge(s_point, node, length=math.dist(self.pos[s_point], self.pos[node]))


        ### Summary
        # print(self.G.nodes)
        # print(self.G.edges)
        self.added_points = s_count
        self.length = self.G.size(weight='length')


        if mode == 'debug' or mode == 'plot':
            print(f'Number of relay nodes: {self.added_points}')
            print(f'Total length: {self.length}\n')

        if mode == 'plot':

            options = {
                # "font_size": 36,
                # "node_size": 3000,
                "node_color": "white",
                "edgecolors": "black",
                # "linewidths": 5,
                # "width": 5,
            }
            nx.draw_networkx(self.G, self.pos, **options)

            # Set margins for the axes so that nodes aren't clipped
            ax = plt.gca()
            ax.margins(0.20)
            ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
            ax.set_aspect('equal')

            plt.show()


if __name__ == "__main__":
    
    # explicitly set positions
    pos = {
        1: (0.4588353, 0.2373239),
        2: (0.1270637, 0.3509955),
        3: (0.1544539, 0.4808225),
        4: (0.9474026, 0.1441120),
        5: (0.2867392, 0.0565889),
        6: (0.1326895, 0.1166049)
    }

    comm_range = 0.25

    solver = ESMT(pos, comm_range)
    solver.solve(mode='plot') # mode = {'release'|'debug'|'plot'}, default is 'release'

    print(f'output num points: {solver.added_points}')
    print(f'output length: {solver.length}')
