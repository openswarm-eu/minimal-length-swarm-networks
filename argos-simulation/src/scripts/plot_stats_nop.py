from os import listdir, environ
from os.path import isdir, join, splitext, dirname, normpath, basename

# Plotting
import numpy as np
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True
import matplotlib.ticker as ticker
from matplotlib.patches import Rectangle, PathPatch
from matplotlib.font_manager import FontProperties
from matplotlib.collections import LineCollection
import seaborn as sns
import pandas as pd
from statannotations.Annotator import Annotator

# Utility
import sys
import math
import time
from collections import defaultdict
from itertools import combinations
import pprint
from copy import deepcopy

# Parse simulation log
sys.path.append(join(dirname(__file__), "..", "protos", "generated")) # Path to compiled proto files
import time_step_pb2
from load_data import SimData

import networkx as nx
from esmt import ESMT
from matplotlib.lines import Line2D

from scipy.spatial.distance import cdist, euclidean
 
from colorama import Fore, Back, Style

# Path to simulation logs
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/nop_ex1/')
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/nop_ex0/')
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/nop_ex2/')
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/real_robots')
RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance/')
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_iros/')
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_comm/')
# RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_real_robots/')

BINARY_FILENAME = 'log_data.pb'
SUMMARY_FILENAME = 'summary.csv'
# SUMMARY_FILENAME = ''
COMMANDS_FILENAME = 'commands.csv'

# Parameters
ROBOT_RADIUS = 0.035
HEADING_LENGTH = 0.05
COMM_RANGE = 0.8 # simulation
# COMM_RANGE = 0.6 # swarmhack

SWARMHACK_X = 0.4 # Fixed distance in the x-direction to subtract from swarmhack log
SWARMHACK_Y = 0.2 # Fixed distance in the y-direction
SWARMHACK_FIXY = 1.09 # Fixed distance to add from swarmhack log after flipping sign
SWARMHACK_ARENA_X = 1.8 # arena height
SWARMHACK_ARENA_Y = 0.9 # arena width

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z # in radians


def geometric_median(X, eps=1e-5):
    y = np.mean(X, 0)

    while True:
        D = cdist(X, [y])
        nonzeros = (D != 0)[:, 0]

        Dinv = 1 / D[nonzeros]
        Dinvs = np.sum(Dinv)
        W = Dinv / Dinvs
        T = np.sum(W * X[nonzeros], 0)

        num_zeros = len(X) - np.sum(nonzeros)
        if num_zeros == 0:
            y1 = T
        elif num_zeros == len(X):
            return y
        else:
            R = (T - y) * Dinvs
            r = np.linalg.norm(R)
            rinv = 0 if r == 0 else num_zeros/r
            y1 = max(0, 1-rinv)*T + min(1, rinv)*y

        if euclidean(y, y1) < eps:
            return y1

        y = y1


def load_log(path):

    # Load log and commands file
    log_file = join(path, BINARY_FILENAME) # path: RESULTS_DIR/scenario/BINARY_FILE
    # commands_file = join(RESULTS_DIR, scenario, COMMANDS_FILENAME) # path: RESULTS_DIR/scenario/COMMANDS_FILE
    summary_file = ''
    if SUMMARY_FILENAME != '':
        summary_file = join(path, SUMMARY_FILENAME) # path: RESULTS_DIR/scenario/SUMMARY_FILE
    s = SimData(log_file, summary_file)
    # s = SimData(log_file) # Real robot experiments

    return s


def load_log_with_checks(path, print_result=False):
    s = load_log(path)

    # Store points scored
    trial_points = s.totalPoints

    # Store team connectivity info
    trial_team_connectivity = []
    for robot in s.data[s.totalTime]['log'].robots:
        if robot.state == time_step_pb2.Robot.FOLLOWER and robot.hopCountTeam == 255:
            # print(robot.hopCountTeam)
            trial_team_connectivity.append({
                'id': robot.name,
                'team_id': robot.teamID,
                'hop_count_team': robot.hopCountTeam
            })

    # Store global connectivity info
    robot_pos = {}
    leader_follower_pos = defaultdict(list)
    team_pos = {}

    # Store robot positions
    for robot in s.data[s.totalTime]['log'].robots:
        robot_pos[robot.name] = robot.position

        if robot.teamID != 255: # Ignore connectors and travellers
            leader_follower_pos[robot.teamID].append(robot.position)
        
        if robot.state == time_step_pb2.Robot.LEADER:
            team_pos[robot.teamID] = {
                'x': robot.position.x,
                'y': robot.position.y
            }

    # # Store team positions
    # for team, positions in leader_follower_pos.items():
    #     num_robots = len(positions)

    #     x = y = 0
    #     for pos in positions:
    #         x += pos.x
    #         y += pos.y

    #     team_pos[team] = {
    #         'x': x / num_robots,
    #         'y': y / num_robots
    #     }

    # init Networkx graph
    G = nx.Graph()
    for team in team_pos.keys():
        G.add_node(team)

    num_connectors = 0
    connections = {}

    # Plot robots
    for robot in s.data[s.totalTime]['log'].robots:
        pos = robot.position

        # Save the connections
        if robot.state == time_step_pb2.Robot.CONNECTOR:
            # print(f'--- {robot.name} --- pos=({robot.position.x}, {robot.position.y})')

            my_id = robot.name
            num_connectors += 1

            for hop in robot.hopCount:
                
                if hop.count > 1:
                    
                    # Connection between two connectors

                    other_id = hop.neighbor
                    key = ''
                    if int(my_id[1:]) < int(other_id[1:]):
                        key += my_id + other_id
                    else:
                        key += other_id + my_id

                    if not key in connections:                        

                        dist = math.dist([robot.position.x, robot.position.y], [robot_pos[hop.neighbor].x, robot_pos[hop.neighbor].y])
                        
                        if dist <= COMM_RANGE:
                            connections[key] = {
                                'x_values': [robot.position.x, robot_pos[hop.neighbor].x],
                                'y_values': [robot.position.y, robot_pos[hop.neighbor].y]
                            }
                            G.add_edge(my_id, other_id, length=dist)
                else:
                    
                    # Connection between a tail connector and a team
                    dist = math.dist([robot.position.x, robot.position.y], [team_pos[hop.teamID]['x'], team_pos[hop.teamID]['y']])
                    if dist <= COMM_RANGE:
                        G.add_edge(my_id, hop.teamID, length=dist)
    # print(G.edges)
    if len(G) == 0:
        connectivity = 0
    else:
        connectivity = nx.is_connected(G)

    trial_connectivity = connectivity

    if print_result:
        print(f'### Points scored ###')
        print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, points: {trial_points}')
        print(f'### GLOBAL Connectivity ###')
        print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, connectivity: {trial_connectivity}')
        print(f'### TEAM Connectivity ###')
        print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, connectivity: {trial_team_connectivity}')
    return s, trial_points, trial_connectivity, trial_team_connectivity


def get_network_length_and_connectors(s, plot=False):
    """Get the network length of produced by the robots and the solver result."""

    # print(s.data[s.totalTime]['log'])

    if plot:
        # Font
        font = FontProperties()
        font.set_family('serif')
        font.set_name('Times New Roman')
        font.set_size(20)

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(16, 8))

        # z-order
        z_arena = 0
        z_task = 1
        z_team_trajectory = 2
        z_connection = 3
        z_team_center = 4
        z_robot_position = 5
        z_robot_orientation = 6

    # sns.scatterplot(data=df, ax = ax, x="PositionX", y="PositionY", hue="State", zorder=2)

    # Store position of robots and sort followers by team
    robot_pos_by_time = {}
    team_pos_by_time = {}

    for time in range(1,s.totalTime+1):

        # Store data every 10 timesteps, the first and final timesteps
        if time % 10 == 0 or time == 1 or time == s.totalTime:

            robot_pos = {}
            leader_follower_pos = defaultdict(list)
            team_pos = {}

            # Store robot positions
            for robot in s.data[time]['log'].robots:
                if time == s.totalTime:
                    robot_pos[robot.name] = robot.position

                if robot.teamID != 255: # Ignore connectors and travellers
                    leader_follower_pos[robot.teamID].append(robot.position)

                # Store leader position
                if robot.name[0] == 'L' or robot.state == time_step_pb2.Robot.LEADER:
                    team_pos[robot.teamID] = {
                        'x': robot.position.x,
                        'y': robot.position.y
                    }

            robot_pos_by_time[time] = robot_pos

            # # Store team positions
            # for team, positions in leader_follower_pos.items():
            #     num_robots = len(positions)

            #     x = y = 0
            #     for pos in positions:
            #         x += pos.x
            #         y += pos.y

            #     team_pos[team] = {
            #         'x': x / num_robots,
            #         'y': y / num_robots
            #     }

            team_pos_by_time[time] = team_pos
    
    num_connectors = 0
    connections = {}

    label_leader = label_follower = label_connector = label_traveler = None

    # init Networkx graph
    G = nx.Graph()
    for team in team_pos_by_time[s.totalTime].keys():
        G.add_node(team)

    # Plot robots
    for robot in s.data[s.totalTime]['log'].robots:

        pos = robot.position
        orientation = robot.orientation

        if plot:
            color = 'white'

            if robot.state == time_step_pb2.Robot.CONNECTOR:
                color = 'cyan'
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                color = 'limegreen'
            elif robot.state == time_step_pb2.Robot.LEADER:
                color = 'red'
            elif robot.state == time_step_pb2.Robot.TRAVELER:
                color = 'magenta'

            # Draw position
            circle_robot = plt.Circle(xy=(pos.x, pos.y), radius=ROBOT_RADIUS, facecolor=color, fill=True, linewidth=1, edgecolor='k', zorder=z_robot_position)
            
            if robot.state == time_step_pb2.Robot.CONNECTOR:
                label_connector = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                label_follower = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.LEADER:
                label_leader = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)

            ax[0].add_patch(circle_robot)

            # Draw orientation
            rad = quaternion_to_euler_angle_vectorized1(orientation.w, orientation.x, orientation.y, orientation.z)[2]
            x_values = [pos.x, pos.x + HEADING_LENGTH * math.cos(rad)]
            y_values = [pos.y, pos.y + HEADING_LENGTH * math.sin(rad)]
            ax[0].plot(x_values, y_values, 'k', zorder=z_robot_orientation)

        # Save the connections
        if robot.state == time_step_pb2.Robot.CONNECTOR:
            # print(f'--- {robot.name} --- pos=({robot.position.x}, {robot.position.y})')

            my_id = robot.name

            num_connectors += 1

            for hop in robot.hopCount:
                
                if hop.count > 1:
                    
                    # Connection between two connectors

                    other_id = hop.neighbor
                    key = ''
                    if int(my_id[1:]) < int(other_id[1:]):
                        key += my_id + other_id
                    else:
                        key += other_id + my_id

                    robot_pos = robot_pos_by_time[s.totalTime]

                    if not key in connections:                        

                        dist = math.dist([robot.position.x, robot.position.y], [robot_pos[hop.neighbor].x, robot_pos[hop.neighbor].y])
                        
                        if dist <= COMM_RANGE:
                            connections[key] = {
                                'x_values': [robot.position.x, robot_pos[hop.neighbor].x],
                                'y_values': [robot.position.y, robot_pos[hop.neighbor].y]
                            }
                            G.add_edge(my_id, other_id, length=dist)
                else:
                    
                    # Connection between a tail connector and a team
                    
                    key = my_id + str(hop.teamID)

                    team_pos = team_pos_by_time[s.totalTime]

                    # print(s.data[s.totalTime])
                    # print(team_pos_by_time[s.totalTime])
                    # print('team_pos:', team_pos)
                    dist = math.dist([robot.position.x, robot.position.y], [team_pos[hop.teamID]['x'], team_pos[hop.teamID]['y']])
                    
                    if dist <= COMM_RANGE:
                        connections[key] = {
                            'x_values': [robot.position.x, team_pos[hop.teamID]['x']],
                            'y_values': [robot.position.y, team_pos[hop.teamID]['y']]
                        }
                        G.add_edge(my_id, hop.teamID, length=dist)
                    else:
                        # If the connector's distance to the team center exceeds the communication range,
                        # Find the follower in that team that is closest to the team center and is within
                        # the connector's communication range.
                        # If that follower is still not in range of the team center, repeat the process.

                        print(f'Connector {my_id} is not close enough ({dist:.2f}) to the team ({hop.teamID}) center...')

                        # current robot
                        # current distance
                        current_robot = deepcopy(robot)
                        prev_robot = deepcopy(robot)
                        current_min_dist = dist
                        isDisconnected = False

                        # while team center not reached by current robot
                        while (current_min_dist != None and current_min_dist > COMM_RANGE) and not isDisconnected:

                            current_min_dist = None # reset

                            # For all robots
                            for other_robot in s.data[s.totalTime]['log'].robots:

                                # If it belongs to the target team
                                if other_robot.teamID == hop.teamID:
                                    dist_between = math.dist([current_robot.position.x, current_robot.position.y], [other_robot.position.x, other_robot.position.y])
                                    
                                    # If the distance from this robot is within the communication range
                                    if dist_between <= COMM_RANGE:
                                        other_dist = math.dist([other_robot.position.x, other_robot.position.y], [team_pos[hop.teamID]['x'], team_pos[hop.teamID]['y']])
                                        
                                        # If distance to team center is smaller than the current smallest distance
                                        if not current_min_dist or other_dist < current_min_dist:
                                            print(f'{other_robot.name} is close to the team center ({other_dist:.2f})')

                                            # Save this robot and distance
                                            current_min_dist = other_dist
                                            current_robot = deepcopy(other_robot)

                            if current_robot.name != prev_robot.name:
                                print(f'Adding {current_robot.name} to the graph')

                                # Add the robot to the graph
                                key = ''
                                if int(current_robot.name[1:]) < int(prev_robot.name[1:]):
                                    key += current_robot.name + prev_robot.name
                                else:
                                    key += prev_robot.name + current_robot.name
                                connections[key] = {
                                    'x_values': [current_robot.position.x, prev_robot.position.x],
                                    'y_values': [current_robot.position.y, prev_robot.position.y]
                                }
                                G.add_edge(current_robot.name, prev_robot.name, length=current_min_dist)

                                prev_robot = current_robot

                            else:
                                print(f'NO ROBOT FOUND TO BE CLOSE TO THE TEAM CENTER! robot: {current_robot.name} @ ({current_robot.position.x:.2f},{current_robot.position.y:.2f}) team: {hop.teamID}')
                                isDisconnected = True
                        
                        if not isDisconnected:
                            # Add the connection between the robot and the team center
                            key = current_robot.name + str(hop.teamID)
                            connections[key] = {
                                'x_values': [current_robot.position.x, team_pos[hop.teamID]['x']],
                                'y_values': [current_robot.position.y, team_pos[hop.teamID]['y']]
                            }
                            G.add_edge(current_robot.name, hop.teamID, length=dist)

    if plot:
        # Draw the center of mass of each team
        for pos in team_pos_by_time[s.totalTime].values():
            ax[0].plot(pos['x'], pos['y'], marker='*', color='orange', markersize=10, zorder=z_team_center)
            label_team = plt.Line2D([], [], marker='*', linestyle='None', color='orange', markersize=10)

        # Draw connections as lines
        for line in connections.values():
            ax[0].plot(line['x_values'], line['y_values'], 'cyan', zorder=z_connection)

        # Plot tasks
        for task in s.data[s.totalTime]['log'].tasks:
            pos = task.position
            circle_task = plt.Circle(xy=(pos.x, pos.y), radius=task.radius, facecolor='whitesmoke', fill=True, linewidth=1, edgecolor='grey', zorder=z_task)
            ax[0].add_patch(circle_task)

        # Reorganize team positions by teams instead of timesteps
        team_trajectory = defaultdict()
        for team in team_pos_by_time[1].keys():
            x_values = []
            y_values = []
            for time, team_pos in team_pos_by_time.items():
                x_values.append(team_pos[team]['x'])
                y_values.append(team_pos[team]['y'])

            team_trajectory[team] = {
                'x': x_values,
                'y': y_values
            }

        # Draw team trajectory
        for team, team_pos in team_trajectory.items():
            ax[0].plot(team_pos['x'], team_pos['y'], linestyle='dashed', zorder=z_team_trajectory)

    # # Run the Steiner tree MSP solver
    # positions = {}
    # for team, pos in team_pos_by_time[s.totalTime].items():
    #     positions[team] = (pos['x'], pos['y'])

    # smt_solver = SMT_MSP(positions, COMM_RANGE)
    # smt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # smt_pos = smt_solver.pos
    
    # # Run the Two tiered relay node placement solver
    # positions = {}
    # for team, pos in team_pos_by_time[s.totalTime].items():
    #     positions[team] = (pos['x'], pos['y'])

    # ttrnp_solver = TwoTieredRNP(positions, COMM_RANGE)
    # ttrnp_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # ttrnp_pos = ttrnp_solver.pos

    # Run the Two tiered relay node placement solver
    positions = {}
    for team, pos in team_pos_by_time[s.totalTime].items():
        positions[team] = (pos['x'], pos['y'])

    esmt_solver = ESMT(positions, COMM_RANGE)
    esmt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    esmt_pos = esmt_solver.pos

    # calculate star toplogy
    positions_star = {}
    for team, pos in team_pos_by_time[time].items():
        # if team != 100000: # Ignore template 100000 connection, which means null
        positions_star[team] = (pos['x'], pos['y'])

    # print('positions_star: ', positions_star)
    # find the centroid of points in positions_star
        
    # centroid = np.mean(list(positions_star.values()), axis=0)
    geometric_median_pos = geometric_median(np.array(list(positions_star.values())))

    # print('centroid: ', centroid)
    # calculate the distance between the centroid and all points in positions_star
    # distances = [math.dist(centroid, pos) for pos in positions_star.values()]
    distances = [math.dist(geometric_median_pos, pos) for pos in positions_star.values()]
    # print('distances: ', distances)
    # find the sum of the distances to the centroid
    star_length = sum(distances)

    if plot:
        for node in esmt_solver.G.nodes:
            if str(node)[0] == 'S':
                circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, linewidth=1, edgecolor='k', zorder=z_robot_position)
                ax[1].add_patch(circle_robot)
            else:
                # ax[1].plot(esmt_pos[node][0], esmt_pos[node][1], marker='*', color='orange', markersize=10, zorder=z_team_center)
                pass

        for edge in esmt_solver.G.edges:
            ax[1].plot([esmt_pos[edge[0]][0], esmt_pos[edge[1]][0]], [esmt_pos[edge[0]][1], esmt_pos[edge[1]][1]], 'cyan', zorder=z_connection)

        ### EXPERIMENT INFO ###
        print('\n------ Experiment result -------')
        print(f'Connectors: {num_connectors}')
        print(f'Network is connected: {nx.is_connected(G)}')
        print(f'Total connection distance: {G.size(weight="length")}')
        print(f'Nodes: {G.nodes}')
        print(f'Edges: {G.edges}')

        print('\n------ ESMT Solver result -------')
        print(f'Connectors: {esmt_solver.added_points}')
        print(f'Network is connected: {nx.is_connected(esmt_solver.G)}')
        print(f'Total connection distance: {esmt_solver.length}')
        print(f'Nodes: {esmt_solver.G.nodes}')
        print(f'Edges: {esmt_solver.G.edges}')

        for i, axis in enumerate(ax):

            # Plot arena 
            circle_arena = plt.Circle(xy=(0, 0), radius=s.arenaRadius, color='black', fill=False, zorder=z_arena)
            axis.add_patch(circle_arena)
        
            if i == 0:
                legend_robot = axis.legend([label_leader, label_follower, label_connector, label_team], ['Leader', 'Follower', 'Connector', 'Team Center'], loc=1)
                axis.set_title(f'Experiment (Connectors = {num_connectors}, Length = {round(G.size(weight="length"), 4)}, Connected = {nx.is_connected(G)})')
            else:
                legend_robot = axis.legend([label_connector, label_team], ['Connector', 'Team Center'], loc=1)
                axis.set_title(f'SMT-MSP Solver (Connectors = {esmt_solver.added_points}, Length = {round(esmt_solver.length, 4)}, Connected = {nx.is_connected(esmt_solver.G)})')

            axis.add_artist(legend_robot)

            # Fit to arena size
            plt.setp(axis, xlim=[-s.arenaRadius,s.arenaRadius])
            plt.setp(axis, ylim=[-s.arenaRadius,s.arenaRadius])
            axis.set_aspect('equal')
            axis.set_xlabel('X (m)', fontproperties=font)
            axis.set_ylabel('Y (m)', fontproperties=font)
            for label in axis.get_xticklabels():
                label.set_fontproperties(font)
            for label in axis.get_yticklabels():
                label.set_fontproperties(font)

        plt.show()

    return {'res': G, 'esmt': esmt_solver.G}, \
           {'res': G.size(weight="length"), 'esmt': esmt_solver.length, 'star': star_length}, \
           {'res': num_connectors, 'esmt': esmt_solver.added_points} 


def get_team_distance(G):
    """Calculate the sum of the distances between every team"""
    team_nodes = [n for n in G.nodes if str(n)[0] != 'S' and str(n)[0] != 'F']
    pairs = combinations(team_nodes, 2)

    dist = 0
    if nx.is_connected(G):
        for v1, v2 in pairs:
            dist += nx.shortest_path_length(G, v1, v2, 'length')
    else:
        print('not connected')
    return dist


def get_team_distances(G):
    """Calculate the min, max and mean distances between teams along the network as well as the difference between the min and max"""
    team_nodes = [n for n in G.nodes if str(n)[0] != 'S' and str(n)[0] != 'F']
    pairs = combinations(team_nodes, 2)

    min = max = None
    sum = mean = 0
    count = 0

    if nx.is_connected(G):
        for v1, v2 in pairs:
            dist = nx.shortest_path_length(G, v1, v2, 'length')

            if not min or dist < min:
                min = dist
            
            if not max or dist > max:
                max = dist

            sum += dist
            count += 1
    else:
        print('not connected')
        return {}
    
    if count > 0:
        mean = sum / count

    return {'min': min, 'max': max, 'mean': mean, 'diff': max - min}


def plot_all_trials():
    
    df = pd.DataFrame({
        'Teams':                        pd.Series(dtype='int'), 
        'Workers Per Team':             pd.Series(dtype='int'),
        'Network Length':               pd.Series(dtype='float'),
        'Network Length (ESMT)':        pd.Series(dtype='float'),
        'Connectors':                   pd.Series(dtype='int'),
        'Connectors (ESMT)':            pd.Series(dtype='int'),
        'Average Path Length':          pd.Series(dtype='float'),
        'Average Path Length (ESMT)':   pd.Series(dtype='float'),          
    })
    
    # Get trial names
    team_num_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    team_num_dirs.sort()

    for team_num in team_num_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, team_num)) if isdir(join(RESULTS_DIR, team_num, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distances(graphs["res"])
            esmt_path_length = get_team_distances(graphs["esmt"])

            # Add data of the successful trial
            if not scenario in trial_no_points and not scenario in trial_no_team_connectivity and not scenario in trial_no_connectivity:

                d = pd.DataFrame({
                    'Teams': [s.numLeaders], 
                    'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
                    'Network Length': [lengths['res']], 
                    'Network Length (ESMT)': [lengths['esmt']],
                    'Connectors': [connectors['res']],
                    'Connectors (ESMT)': [connectors['esmt']],
                    'Average Path Length': [res_path_length['mean']],
                    'Average Path Length (ESMT)': [esmt_path_length['mean']],
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            num_to_use = 50
            if (count % 10) % num_to_use == 0:
                count += 50 - num_to_use
                # print(count)
                # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig1 = plt.figure(figsize=(6, 5))
    axes1 = fig1.gca()
    fig2 = plt.figure(figsize=(6, 5))
    axes2 = fig2.gca()
    fig3 = plt.figure(figsize=(6, 5))
    axes3 = fig3.gca()
    fig4 = plt.figure(figsize=(6, 5))
    axes4 = fig4.gca()
    fig5 = plt.figure(figsize=(6, 5))
    axes5 = fig5.gca()
    fig6 = plt.figure(figsize=(6, 5))
    axes6 = fig6.gca()
    # fig7 = plt.figure(figsize=(9, 5))
    # axes7 = fig7.gca()
    # fig8 = plt.figure(figsize=(9, 5))
    # axes8 = fig8.gca()

    fig9, axd9 = plt.subplot_mosaic([['ul', 'r'],
                                   ['ml', 'r'],
                                   ['bl', 'r']],
                                   figsize=(9,12),
                                   gridspec_kw={'width_ratios': [30, 1]})

    axes = [axes1, axes2, axes3, axes4, axes5, axes6]

    order_num_team = list(df['Teams'].unique())
    order_num_team.sort()
    order_team_size = list(df['Workers Per Team'].unique())
    order_team_size.sort()

    ### LINE PLOT

    # Get subset of dataframe
    df_connectors = df[['Teams', 'Workers Per Team', 'Connectors', 'Connectors (ESMT)']]
    df_connectors = df_connectors.rename(columns={'Connectors': 'Trials', 'Connectors (ESMT)': 'Optimal Trees'})
    df_connectors = df_connectors.melt(id_vars=['Teams', 'Workers Per Team'])#.sort_values(['Workers Per Team', 'Teams', 'variable'], ascending = [True, True, True])
    df_connectors.rename(columns={'variable': 'Type'}, inplace=True)
    # print(df_connectors)
    df_network_length = df[['Teams', 'Workers Per Team', 'Network Length', 'Network Length (ESMT)']]
    df_network_length = df_network_length.rename(columns={'Network Length': 'Trials', 'Network Length (ESMT)': 'Optimal Trees'})
    df_network_length = df_network_length.melt(id_vars=['Teams', 'Workers Per Team'])#.sort_values(['Workers Per Team', 'Teams', 'variable'], ascending = [True, True, True])
    df_network_length.rename(columns={'variable': 'Type'}, inplace=True)
    # print(df_network_length)
    df_team_length = df[['Teams', 'Workers Per Team', 'Average Path Length', 'Average Path Length (ESMT)']]
    df_team_length = df_team_length.rename(columns={'Average Path Length': 'Trials', 'Average Path Length (ESMT)': 'Optimal Trees'})
    df_team_length = df_team_length.melt(id_vars=['Teams', 'Workers Per Team'])#.sort_values(['Workers Per Team', 'Teams', 'variable'], ascending = [True, True, True])
    df_team_length.rename(columns={'variable': 'Type'}, inplace=True)
    # print(df_team_length)

    # Build hue label
    # hue1 = df_network_length[['Workers Per Team', 'Type']].apply(lambda row: f"{row['Workers Per Team']}, {row['Type']}", axis=1)
    # hue2 = df_network_length[['Teams', 'Type']].apply(lambda row: f"{row['Teams']}, {row['Type']}", axis=1)

    sns.lineplot(
        data=df_connectors, ax=axes1, 
        x='Teams', y='value', 
        hue='Workers Per Team',
        err_style='bars', style='Type', dashes=True, palette=['blue','green','orange','red']
    )

    sns.lineplot(
        data=df_connectors, ax=axes2, 
        x='Workers Per Team', y='value', 
        hue='Teams',
        err_style='bars', style='Type', dashes=True, palette=['purple','blue','green','orange','red']
    )

    sns.lineplot(
        data=df_network_length, ax=axes3, 
        x='Teams', y='value', 
        hue='Workers Per Team',
        err_style='bars', style='Type', dashes=True, palette=['blue','green','orange','red']
    )

    sns.lineplot(
        data=df_network_length, ax=axes4, 
        x='Workers Per Team', y='value', 
        hue='Teams',
        err_style='bars', style='Type', dashes=True, palette=['purple','blue','green','orange','red']
    )

    sns.lineplot(
        data=df_team_length, ax=axes5, 
        x='Teams', y='value', 
        hue='Workers Per Team',
        err_style='bars', style='Type', dashes=True, palette=['blue','green','orange','red']
    )

    sns.lineplot(
        data=df_team_length, ax=axes6, 
        x='Workers Per Team', y='value', 
        hue='Teams',
        err_style='bars', style='Type', dashes=True, palette=['purple','blue','green','orange','red']
    )

    ### HEAT MAP

    mean_connectors = df_connectors.groupby(['Teams', 'Workers Per Team', 'Type']).mean().reset_index()
    # print(mean_connectors)
    mean_network_length = df_network_length.groupby(['Teams', 'Workers Per Team', 'Type']).mean().reset_index()
    # print(mean_network_length)
    mean_team_length = df_team_length.groupby(['Teams', 'Workers Per Team', 'Type']).mean().reset_index()

    def calculate_ratio(group, type_col, type1, type2, val_col):
        ratio = group.loc[group[type_col] == type1, val_col].iloc[0] / group.loc[group[type_col] == type2, val_col].iloc[0]
        return pd.Series({'ratio': ratio})

    # group DataFrame by id1 and id2 and apply function to calculate ratio
    ratio_df_connectors = mean_connectors.groupby(['Teams', 'Workers Per Team']).apply(calculate_ratio, type_col='Type', type1='Trials', type2='Optimal Trees', val_col='value')
    print(ratio_df_connectors)
    ratio_df_network_length = mean_network_length.groupby(['Teams', 'Workers Per Team']).apply(calculate_ratio, type_col='Type', type1='Trials', type2='Optimal Trees', val_col='value')
    print(ratio_df_network_length)
    ratio_df_team_length = mean_team_length.groupby(['Teams', 'Workers Per Team']).apply(calculate_ratio, type_col='Type', type1='Trials', type2='Optimal Trees', val_col='value')
    print(ratio_df_team_length)

    pivot_table_connectors = ratio_df_connectors.pivot_table(index='Workers Per Team', columns='Teams', values='ratio')
    pivot_table_network_length = ratio_df_network_length.pivot_table(index='Workers Per Team', columns='Teams', values='ratio')
    pivot_table_team_length = ratio_df_team_length.pivot_table(index='Workers Per Team', columns='Teams', values='ratio')

    # create a list of the desired order of the index values
    index_order = df.sort_values('Workers Per Team', ascending=False)['Workers Per Team'].unique().tolist()

    # reindex the pivot table using the desired order of the index values
    pivot_table_connectors = pivot_table_connectors.reindex(index_order)
    pivot_table_network_length = pivot_table_network_length.reindex(index_order)
    pivot_table_team_length = pivot_table_team_length.reindex(index_order)

    # plot the pivot table as a heatmap using seaborn
    max_disp = max((1 - ratio_df_connectors['ratio']).abs().max(), (1 - ratio_df_network_length['ratio']).abs().max(), (1 - ratio_df_team_length['ratio']).abs().max())
    # sns.heatmap(pivot_table_network_length, ax=axes7, annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp, cbar_kws={'label': 'Ratio'})
    # sns.heatmap(pivot_table_connectors, ax=axes8, annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp, cbar_kws={'label': 'Ratio'})

    sns.heatmap(pivot_table_connectors, ax=axd9['ml'], annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp,
                cbar=True, cbar_ax=axd9['r'])

    sns.heatmap(pivot_table_network_length, ax=axd9['ul'], annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp,
                cbar=False, cbar_ax=None)
    
    sns.heatmap(pivot_table_team_length, ax=axd9['bl'], annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp,
                cbar=False, cbar_ax=None)

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(14)

    # Set axis labels
    # axes.set_xlabel("Average waiting time per task (s)")
    axes1.set_xlabel("Teams", fontproperties=font)
    axes1.set_ylabel("Connectors", fontproperties=font)
    axes2.set_xlabel("Workers Per Team", fontproperties=font)
    axes2.set_ylabel("Connectors", fontproperties=font)
    axes3.set_xlabel("Teams", fontproperties=font)
    axes3.set_ylabel("Network Length (m)", fontproperties=font)
    axes4.set_xlabel("Workers Per Team", fontproperties=font)
    axes4.set_ylabel("Network Length (m)", fontproperties=font)
    axes5.set_xlabel("Teams", fontproperties=font)
    axes5.set_ylabel("Average Path Length (m)", fontproperties=font)
    axes6.set_xlabel("Workers Per Team", fontproperties=font)
    axes6.set_ylabel("Average Path Length (m)", fontproperties=font)
    # axes7.set_xlabel("Teams", fontproperties=font)
    # axes7.set_ylabel("Workers Per Team", fontproperties=font)
    # axes8.set_xlabel("Teams", fontproperties=font)
    # axes8.set_ylabel("Workers Per Team", fontproperties=font)

    for key, ax in axd9.items():
        if key != 'r':
            ax.set_xlabel("Teams", fontproperties=font)
            ax.set_ylabel("Workers Per Team", fontproperties=font)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

    # border and ticks
    ticks = np.arange(2, 6+1, 1)
    axes1.set_xticks(ticks)
    axes1.set_ylim([-1,df_connectors['value'].max()+1])

    ticks = np.arange(6, 24+1, 6)
    axes2.set_xticks(ticks)
    axes2.set_ylim([-1,df_connectors['value'].max()+1])

    ticks = np.arange(2, 6+1, 1)
    axes3.set_xticks(ticks)
    axes3.set_ylim([-1,df_network_length['value'].max()+1])

    ticks = np.arange(6, 24+1, 6)
    axes4.set_xticks(ticks)
    axes4.set_ylim([-1,df_network_length['value'].max()+1])

    ticks = np.arange(2, 6+1, 1)
    axes5.set_xticks(ticks)
    axes5.set_ylim([-1,df_team_length['value'].max()+1])

    ticks = np.arange(6, 24+1, 6)
    axes6.set_xticks(ticks)
    axes6.set_ylim([-1,df_team_length['value'].max()+1])

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (ESMT)', 'w = 12', 'w = 12 (ESMT)', 'w = 18', 'w = 18 (ESMT)', 'w = 24', 'w = 24 (ESMT)']
    # legend_labels2 = ['t = 2', 't = 2 (ESMT)', 't = 3', 't = 3 (ESMT)', 't = 4', 't = 4 (ESMT)', 't = 5', 't = 5 (ESMT)', 't = 6', 't = 6 (ESMT)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes2.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes3.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes4.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes5.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes6.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    # Adjust legends
    for ax in axes:
        leg = ax.legend(loc='upper left')
        for text_obj in leg.get_texts():
            text = text_obj.get_text()
            if text == 'Workers Per Team' or text == 'Teams' or text == 'Type':
                text_obj.set_fontweight('bold')
                text_obj.set_position((-20,0))

    # set font properties of colorbar label
    # cbar7 = axes7.collections[0].colorbar
    # cbar7.set_label("Ratio", fontdict={'size': 14})
    # cbar7.ax.tick_params(labelsize=14)
    # cbar8 = axes8.collections[0].colorbar
    # cbar8.set_label("Ratio", fontdict={'size': 14})
    # cbar8.ax.tick_params(labelsize=14)

    # cbar8 = axd9['r'].collections[0].colorbar
    axd9['r'].set_ylabel("Ratio", fontdict={'size': 14})
    axd9['r'].tick_params(labelsize=14)

    for key, ax in axd9.items():
        if key == 'r':
            ax.spines['top'].set_visible(True)
            ax.spines['right'].set_visible(True)
        else:
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

    axd9['r'].set_ylabel('Ratio')

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()
    fig6.tight_layout()
    # fig7.tight_layout()
    # fig8.tight_layout()

    fig9.tight_layout()
    fig9.subplots_adjust(wspace=0.1)

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'connectors(num_teams).pdf'))
    fig2.savefig(join(RESULTS_DIR, 'connectors(num_robots).pdf'))
    fig3.savefig(join(RESULTS_DIR, 'network_length(num_teams).pdf'))
    fig4.savefig(join(RESULTS_DIR, 'network_length(num_robots).pdf'))
    fig5.savefig(join(RESULTS_DIR, 'average_path_length(num_teams).pdf'))
    fig6.savefig(join(RESULTS_DIR, 'average_path_length(num_robots).pdf'))               
    # fig7.savefig(join(RESULTS_DIR, 'heatmap_network_length.pdf'))               
    # fig8.savefig(join(RESULTS_DIR, 'heatmap_connectors.pdf'))    
    fig9.savefig(join(RESULTS_DIR, 'heatmap.pdf'))    


def get_network_length_and_connectors_at_time(s, time=0, plot=False, real_robot=False):
    """Get the network length of produced by the robots, the number of connectors and the solver result at the specified timestep."""

    if plot:
        # Font
        font = FontProperties()
        # font.set_family('serif')
        # font.set_name('Times New Roman')
        font.set_size(32)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 12))

        # z-order
        z_arena = 0
        z_task = 1
        z_team_trajectory = 2
        z_connection = 3
        z_team_center = 4
        z_robot_position = 5
        z_robot_orientation = 6

    # sns.scatterplot(data=df, ax = ax, x="PositionX", y="PositionY", hue="State", zorder=2)

    # Store position of robots and sort followers by team
    robot_pos_by_time = {}
    team_pos_by_time = {}

    robot_pos = {}
    leader_follower_pos = defaultdict(list)
    team_pos = {}

    # Store robot positions
    for robot in s.data[time]['log'].robots:

        robot_pos[robot.name] = robot.position

        if not real_robot:

            ### Simulation
            if robot.teamID != 255: # Ignore connectors and travellers
                leader_follower_pos[robot.teamID].append(robot.position)

            # Store leader position
            if robot.name[0] == 'L':
                team_pos[robot.teamID] = {
                    'x': robot.position.x,
                    'y': robot.position.y
                }
        else:
            ### Swarmhack fix y-axis
            robot_pos[robot.name].x -= SWARMHACK_X
            robot_pos[robot.name].y = -robot_pos[robot.name].y + SWARMHACK_FIXY

            if robot.teamID != 255 and robot.teamID != 100000: # Ignore connectors and travellers, Ignore swarmhack template team
                leader_follower_pos[robot.teamID].append(robot.position)

            if robot.state == time_step_pb2.Robot.LEADER:
                team_pos[robot.teamID] = {
                    'x': robot.position.x,
                    'y': robot.position.y
                }

    robot_pos_by_time[time] = robot_pos

    # # Store team positions
    # for team, positions in leader_follower_pos.items():
    #     num_robots = len(positions)

    #     x = y = 0
    #     for pos in positions:
    #         x += pos.x
    #         y += pos.y

    #     team_pos[team] = {
    #         'x': x / num_robots,
    #         'y': y / num_robots
    #     }

    team_pos_by_time[time] = team_pos
    
    num_connectors = 0
    connections = {}

    label_leader = label_follower = label_connector = label_traveler = None

    # init Networkx graph
    G = nx.Graph()
    for team in team_pos_by_time[time].keys():
        # if team != 100000: # Ignore template 100000 connection, which means null
        G.add_node(team)

    team_task_pair = {}
    for task in s.data[time]['log'].tasks:
        # print(task.name)
        # print(task.position)

        # loop robots to find the closest
        min_dist = 100000
        closest_team = None
        for robot in s.data[time]['log'].robots:
            if robot.state == time_step_pb2.Robot.LEADER:
                dist = math.dist([robot.position.x, robot.position.y], [task.position.x, task.position.y])
                if dist < min_dist:
                    min_dist = dist
                    closest_team = robot.teamID

        # tasks_leader_pair[task.name] = closest_leader
        team_task_pair[closest_team] = task.position

    print(team_task_pair)

    # Plot robots
    for robot in s.data[time]['log'].robots:

        pos = robot.position
        orientation = robot.orientation

        if plot:
            color = 'white'

            if robot.state == time_step_pb2.Robot.CONNECTOR:
                color = 'cyan'
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                color = 'limegreen'
            elif robot.state == time_step_pb2.Robot.LEADER:
                color = 'red'
            elif robot.state == time_step_pb2.Robot.TRAVELER:
                color = 'magenta'

            # Draw position
            circle_robot = plt.Circle(xy=(pos.x, pos.y), radius=ROBOT_RADIUS, facecolor=color, fill=True, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
            
            if robot.state == time_step_pb2.Robot.CONNECTOR:
                label_connector = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=24)
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                label_follower = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=24)
            elif robot.state == time_step_pb2.Robot.LEADER:
                label_leader = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=24)
            elif robot.state == time_step_pb2.Robot.TRAVELER:
                label_traveler = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=24)

            ax.add_patch(circle_robot)

            # Draw orientation
            rad = quaternion_to_euler_angle_vectorized1(orientation.w, orientation.x, orientation.y, orientation.z)[2]
            # print(f'robot: {robot.name}, rot: {rad}')
            x_values = [pos.x, pos.x + HEADING_LENGTH * math.cos(rad)]
            y_values = [pos.y, pos.y + HEADING_LENGTH * math.sin(rad)]
            ax.plot(x_values, y_values, 'k', zorder=z_robot_orientation, linewidth=1)

        # Save the connections
        if robot.state == time_step_pb2.Robot.CONNECTOR:
            # if robot.name == 'F60':
            #     print(f'--- {robot.name} --- pos=({robot.position.x}, {robot.position.y})')

            my_id = robot.name

            num_connectors += 1

            for hop in robot.hopCount:
                
                tmp_hop = deepcopy(hop)

                if tmp_hop.count > 1:
                    
                    # Connection between two connectors

                    other_id = tmp_hop.neighbor
                    key = ''
                    if int(my_id[1:]) < int(other_id[1:]):
                        key += my_id + other_id
                    else:
                        key += other_id + my_id

                    robot_pos = robot_pos_by_time[time]

                    if not key in connections:                        

                        # Check if whether it is still in the process of changing connection
                        usePrevHop = False

                        if tmp_hop.count == 2 or robot.prevHops:
                            for prevHop in robot.prevHops:
                                # print(f'prevHop: team: {prevHop.teamID}, id: {prevHop.neighbor}, count: {prevHop.count}')
                                for other_robot in s.data[time]['log'].robots:
                                    if other_robot.name == other_id and other_robot.state != time_step_pb2.Robot.CONNECTOR:
                                        print(f'Connection does not exist! reverting back to {prevHop.neighbor}')
                                        tmp_hop = deepcopy(prevHop)
                                        usePrevHop = True

                        if not usePrevHop:
                            dist = math.dist([robot.position.x, robot.position.y], [robot_pos[tmp_hop.neighbor].x, robot_pos[tmp_hop.neighbor].y])
                            
                            if dist <= COMM_RANGE:
                                connections[key] = {
                                    'x_values': [robot.position.x, robot_pos[tmp_hop.neighbor].x],
                                    'y_values': [robot.position.y, robot_pos[tmp_hop.neighbor].y]
                                }
                                G.add_edge(my_id, other_id, length=dist)

                if tmp_hop.count == 1:
                    
                    # Connection between a tail connector and a team
                    
                    key = my_id + str(tmp_hop.teamID)

                    team_pos = team_pos_by_time[time]

                    dist = math.dist([robot.position.x, robot.position.y], [team_pos[tmp_hop.teamID]['x'], team_pos[tmp_hop.teamID]['y']])
                    
                    if dist <= COMM_RANGE:
                        connections[key] = {
                            'x_values': [robot.position.x, team_pos[tmp_hop.teamID]['x']],
                            'y_values': [robot.position.y, team_pos[tmp_hop.teamID]['y']]
                        }
                        G.add_edge(my_id, tmp_hop.teamID, length=dist)
                    else:
                        # If the connector's distance to the team center exceeds the communication range,
                        # Find the follower in that team that is closest to the team center and is within
                        # the connector's communication range.
                        # If that follower is still not in range of the team center, repeat the process.

                        print(f'Connector {my_id} is not close enough ({dist:.2f}) to the team ({tmp_hop.teamID}) center...')

                        # current robot
                        # current distance
                        current_robot = deepcopy(robot)
                        prev_robot = deepcopy(robot)
                        current_min_dist = dist
                        isDisconnected = False

                        # while team center not reached by current robot
                        while (current_min_dist != None and current_min_dist > COMM_RANGE) and not isDisconnected:

                            current_min_dist = None # reset

                            # For all robots
                            for other_robot in s.data[time]['log'].robots:

                                # If it belongs to the target team
                                if other_robot.teamID == tmp_hop.teamID:
                                    dist_between = math.dist([current_robot.position.x, current_robot.position.y], [other_robot.position.x, other_robot.position.y])
                                    
                                    # If the distance from this robot is within the communication range
                                    if dist_between <= COMM_RANGE:
                                        other_dist = math.dist([other_robot.position.x, other_robot.position.y], [team_pos[tmp_hop.teamID]['x'], team_pos[tmp_hop.teamID]['y']])
                                        
                                        # If distance to team center is smaller than the current smallest distance
                                        if not current_min_dist or other_dist < current_min_dist:
                                            print(f'{other_robot.name} is close to the team center ({other_dist:.2f})')

                                            # Save this robot and distance
                                            current_min_dist = other_dist
                                            current_robot = deepcopy(other_robot)

                            if current_robot.name != prev_robot.name:
                                print(f'Adding {current_robot.name} to the graph')

                                # Add the robot to the graph
                                key = ''
                                if int(current_robot.name[1:]) < int(prev_robot.name[1:]):
                                    key += current_robot.name + prev_robot.name
                                else:
                                    key += prev_robot.name + current_robot.name
                                connections[key] = {
                                    'x_values': [current_robot.position.x, prev_robot.position.x],
                                    'y_values': [current_robot.position.y, prev_robot.position.y]
                                }
                                G.add_edge(current_robot.name, prev_robot.name, length=current_min_dist)

                                prev_robot = current_robot

                            else:
                                print(f'NO ROBOT FOUND TO BE CLOSE TO THE TEAM CENTER! robot: {current_robot.name} @ ({current_robot.position.x:.2f},{current_robot.position.y:.2f}) team: {hop.teamID}')
                                isDisconnected = True
                        
                        if not isDisconnected:
                            # Add the connection between the robot and the team center
                            key = current_robot.name + str(tmp_hop.teamID)
                            connections[key] = {
                                'x_values': [current_robot.position.x, team_pos[tmp_hop.teamID]['x']],
                                'y_values': [current_robot.position.y, team_pos[tmp_hop.teamID]['y']]
                            }
                            G.add_edge(current_robot.name, tmp_hop.teamID, length=dist)

    if plot:
        # # Draw the center of mass of each team
        # for pos in team_pos_by_time[time].values():
        #     ax.plot(pos['x'], pos['y'], marker='*', color='orange', markersize=16, zorder=z_team_center)
        #     label_team = plt.Line2D([], [], marker='*', linestyle='None', color='orange', markersize=24)

        # Draw connections as lines
        # Get the second color in palette 'Set1'
        for line in connections.values():
            ax.plot(line['x_values'], line['y_values'], color=sns.color_palette('Set2')[1], zorder=z_connection, linewidth=3)

        # Plot tasks
        for task in s.data[time]['log'].tasks:
            pos = task.position
            if not real_robot:
                ### Simulation
                pass
            else:
                ### Swarmhack
                pos.x -= SWARMHACK_X
                pos.y = -pos.y + SWARMHACK_FIXY
            circle_task = plt.Circle(xy=(pos.x, pos.y), radius=task.radius, facecolor='whitesmoke', fill=True, linewidth=1, edgecolor='grey', zorder=z_task)
            ax.add_patch(circle_task)
            # ax.plot(pos.x, pos.y, marker='*', color='orange', markersize=16, zorder=z_team_center)
            # label_team = plt.Line2D([], [], marker='*', linestyle='None', color='orange', markersize=24)

        # Reorganize team positions by teams instead of timesteps
        team_trajectory = defaultdict()
        for team in team_pos_by_time[time].keys():
            x_values = []
            y_values = []
            for time, team_pos in team_pos_by_time.items():
                x_values.append(team_pos[team]['x'])
                y_values.append(team_pos[team]['y'])

            team_trajectory[team] = {
                'x': x_values,
                'y': y_values
            }

        print(team_trajectory)

        # # Draw team trajectory
        # for team, team_pos in team_trajectory.items():
        #     ax.plot(team_task_pair[team].x, team_pos['y'], linestyle='dashed', zorder=z_team_trajectory)
        #     # ax.plot(team_task_pair[team].x, team_task_pair[team].y, linestyle='dashed', zorder=z_team_trajectory)

    ### Solvers

    # # 1) Run the SMT_MSP solver
    # positions = {}
    # for team, pos in team_pos_by_time[time].items():
    #     # if team != 100000: # Ignore template 100000 connection, which means null
    #     positions[team] = (pos['x'], pos['y'])

    # # print(f'positions {positions}')

    # smt_solver = SMT_MSP(positions, COMM_RANGE)
    # smt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # smt_pos = smt_solver.pos
    
    # # 2) Run the TwoTieredRNP solver
    # positions = {}
    # for team, pos in team_pos_by_time[time].items():
    #     # if team != 100000: # Ignore template 100000 connection, which means null
    #     positions[team] = (pos['x'], pos['y'])

    # # print(f'positions {positions}')

    # ttrnp_solver = TwoTieredRNP(positions, COMM_RANGE)
    # ttrnp_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # ttrnp_pos = ttrnp_solver.pos

    # Run the ESMT solver
    positions = {}
    for team, pos in team_pos_by_time[time].items():
        # if team != 100000: # Ignore template 100000 connection, which means null
        positions[team] = (pos['x'], pos['y'])
        # positions[team] = (team_task_pair[team].x, team_task_pair[team].y)

    # print(f'positions {positions}')

    esmt_solver = ESMT(positions, COMM_RANGE)
    esmt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    esmt_pos = esmt_solver.pos

    if plot:
        # SMT
        # for node in smt_solver.G.nodes:
        #     if str(node)[0] == 'S':
        #         circle_robot = plt.Circle(xy=(smt_pos[node][0], smt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
        #         ax[1].add_patch(circle_robot)
        #     else:
        #         ax[1].plot(smt_pos[node][0], smt_pos[node][1], marker='*', color='orange', markersize=10, zorder=z_team_center)

        # for edge in smt_solver.G.edges:
        #     ax[1].plot([smt_pos[edge[d0]][0], smt_pos[edge[1]][0]], [smt_pos[edge[0]][1], smt_pos[edge[1]][1]], 'cyan', zorder=z_connection)

        # for node in esmt_solver.G.nodes:
        #     if str(node)[0] == 'S':
        #         # circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
        #         # ax[1].add_patch(circle_robot)
        #         pass
        #     else:
        #         ax.plot(esmt_pos[node][0], esmt_pos[node][1], marker='*', color='orange', markersize=16, zorder=z_team_center)
        #         # if esmt_solver.G.degree[node] > 1:
        #         #     circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, alpha=0.3, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
        #         #     ax[1].add_patch(circle_robot)

        for edge in esmt_solver.G.edges:
            ax.plot([esmt_pos[edge[0]][0], esmt_pos[edge[1]][0]], [esmt_pos[edge[0]][1], esmt_pos[edge[1]][1]], color=sns.color_palette('Set2')[0], zorder=z_connection, linewidth=3)

        # # calculate star toplogy
        # positions_star = {}
        # for team, pos in team_pos_by_time[time].items():
        #     # if team != 100000: # Ignore template 100000 connection, which means null
        #     positions_star[team] = (pos['x'], pos['y'])
        #     # positions_star[team] = (team_task_pair[team].x, team_task_pair[team].y)

        # print('positions_star: ', positions_star)
        # # find the centroid of points in positions_star
        # centroid = np.mean(list(positions_star.values()), axis=0)
        # print('centroid: ', centroid)
        # # calculate the distance between the centroid and all points in positions_star
        # distances = [math.dist(centroid, pos) for pos in positions_star.values()]
        # print('distances: ', distances)
        # # find the sum of the distances to the centroid
        # sum_distances = sum(distances)

        # calculate star toplogy
        positions_star = {}
        for team, pos in team_pos_by_time[time].items():
            # if team != 100000: # Ignore template 100000 connection, which means null
            positions_star[team] = (pos['x'], pos['y'])

        # print('positions_star: ', positions_star)
        # find the centroid of points in positions_star
            
        # centroid = np.mean(list(positions_star.values()), axis=0)
        geometric_median_pos = geometric_median(np.array(list(positions_star.values())))

        # print('centroid: ', centroid)
        # calculate the distance between the centroid and all points in positions_star
        # distances = [math.dist(centroid, pos) for pos in positions_star.values()]
        distances = [math.dist(geometric_median_pos, pos) for pos in positions_star.values()]
        # print('distances: ', distances)
        # find the sum of the distances to the centroid
        star_length = sum(distances)

        # create a dict of lines connecting the centroid to each team
        connections_star = {}
        for team, pos in positions_star.items():
            connections_star[team] = {
                'x_values': [pos[0], geometric_median_pos[0]],
                'y_values': [pos[1], geometric_median_pos[1]]
            }

        for line in connections_star.values():
            ax.plot(line['x_values'], line['y_values'], color=sns.color_palette('Set2')[2], zorder=z_connection, linewidth=3)

        ### EXPERIMENT INFO ###
        print('\n------ Experiment result -------')
        print(f'Connectors: {num_connectors}')
        print(f'Network is connected: {nx.is_connected(G)}')
        print(f'Total connection distance: {G.size(weight="length")}')
        print(f'Nodes: {G.nodes}')
        print(f'Edges: {G.edges}')

        # print('\n------ SMT-MSP Solver result -------')
        # print(f'Connectors: {smt_solver.added_points}')
        # print(f'Network is connected: {nx.is_connected(smt_solver.G)}')
        # print(f'Total connection distance: {smt_solver.length}')
        # print(f'Nodes: {smt_solver.G.nodes}')
        # print(f'Edges: {smt_solver.G.edges}')

        # print('\n------ TwoTieredRNP Solver result -------')
        # print(f'Connectors: {ttrnp_solver.added_points}')
        # print(f'Network is connected: {nx.is_connected(ttrnp_solver.G)}')
        # print(f'Total connection distance: {ttrnp_solver.length}')
        # print(f'Nodes: {ttrnp_solver.G.nodes}')
        # print(f'Edges: {ttrnp_solver.G.edges}')

        print('\n------ ESMT Solver result -------')
        print(f'Connectors: {esmt_solver.added_points}')
        print(f'Network is connected: {nx.is_connected(esmt_solver.G)}')
        print(f'Total connection distance: {esmt_solver.length}')
        print(f'Nodes: {esmt_solver.G.nodes}')
        print(f'Edges: {esmt_solver.G.edges}')

        print('\n------ Star topology result -------')
        # print(f'Connectors: {esmt_solver.added_points}')
        # print(f'Network is connected: {nx.is_connected(esmt_solver.G)}')
        print(f'Total connection distance: {star_length}')
        # print(f'Nodes: {esmt_solver.G.nodes}')
        # print(f'Edges: {esmt_solver.G.edges}')

        # for i, axis in enumerate(ax):

        # # Plot arena 
        # if not real_robot:
        #     circle_arena = plt.Circle(xy=(0, 0), radius=s.arenaRadius, color='black', fill=False, zorder=z_arena)
        #     ax.add_patch(circle_arena)
    
        # if i == 0:
        ### With traveler
        # legend position to the right
        
        legend_font = '26.5'

        # legend_robot = ax.legend([label_leader, label_follower, label_connector, label_traveler, label_team], ['Leader', 'Follower', 'Connector', 'Traveler', 'Team Center'], loc=1, fontsize='18')
        # legend_robot = ax.legend([None, label_leader, label_follower, label_connector, label_traveler, label_team], ['Roles','Leader', 'Follower', 'Connector', 'Traveler', 'Team Centre'], loc='lower left', bbox_to_anchor=(0, 0.2), fontsize=legend_font)
        if not real_robot:
            legend_robot = ax.legend([None, label_leader, label_follower, label_connector, label_traveler], ['Roles','Lead robot', 'Follower', 'Connector', 'Traveler'], loc='lower left', bbox_to_anchor=(0, 0.16), fontsize=legend_font)
        else:
            legend_robot = ax.legend([None, label_leader, label_follower, label_connector, label_traveler], ['Roles','Lead robot', 'Follower', 'Connector', 'Traveler'], loc='lower left', bbox_to_anchor=(0.4, 1), fontsize=legend_font)

        # Add a second legend representing the color of the lines which is the 'Solution'
        
        # Get legend_elements from lines drawn using plt.plot()
        legend_elements = [Line2D([0], [0], color=sns.color_palette('Set2')[0], lw=3, label='Steiner tree'),
                           Line2D([0], [0], color=sns.color_palette('Set2')[1], lw=3, label='Our approach'),
                           Line2D([0], [0], color=sns.color_palette('Set2')[2], lw=3, label='Optimal starlike tree')]
        if not real_robot:
            legend_solution = ax.legend(handles=legend_elements, loc='lower left', bbox_to_anchor=(0, 0), fontsize=legend_font)
        else:
            legend_solution = ax.legend(handles=legend_elements, loc='lower left', bbox_to_anchor=(0, 1), fontsize=legend_font)
        ### No traveler
        # legend_robot = axis.legend([label_leader, label_follower, label_connector, label_team], ['Leader', 'Follower', 'Connector', 'Team Center'], loc=1, fontsize='14')
        # axis.set_title(f'Experiment (Connectors = {num_connectors}, Length = {round(G.size(weight="length"), 4)}, Connected = {nx.is_connected(G)})')
        # else:
        #     # # SMT
        #     # legend_robot = axis.legend([label_connector, label_team], ['Connector', 'Team Center'], loc=1)
        #     # axis.set_title(f'STP-MSPBEL (Connectors = {smt_solver.added_points}, Length = {round(smt_solver.length, 4)}, Connected = {nx.is_connected(smt_solver.G)})')

        #     # 2tRNP
        #     legend_robot = ax.legend([label_connector, label_team], ['Connector', 'Team Center'], loc=1, fontsize='20')
        #     # axis.set_title(f'2tRNP (Connectors = {ttrnp_solver.added_points}, Length = {round(ttrnp_solver.length, 4)}, Connected = {nx.is_connected(ttrnp_solver.G)})')

        ax.add_artist(legend_robot)
        ax.add_artist(legend_solution)

        # Fit to arena size
        if not real_robot:
            ### Draw simulation arena
            # print('arena radius: ',s.arenaRadius)
            plt.setp(ax, xlim=[-s.arenaRadius,s.arenaRadius])
            plt.setp(ax, ylim=[-s.arenaRadius,s.arenaRadius])
            plt.subplots_adjust(left=0.1)
        else:
            ### Draw swarmhack arena
            # ax.add_patch(Rectangle((SWARMHACK_X, SWARMHACK_Y), SWARMHACK_ARENA_X, SWARMHACK_ARENA_Y, facecolor='none', ec='k', lw=2))
            ax.add_patch(Rectangle((0, -SWARMHACK_Y+SWARMHACK_FIXY-SWARMHACK_ARENA_Y), SWARMHACK_ARENA_X, SWARMHACK_ARENA_Y, facecolor='none', ec='k', lw=2))
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)', fontproperties=font)
        ax.set_ylabel('Y (m)', fontproperties=font)
        for label in ax.get_xticklabels():
            label.set_fontproperties(font)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font)

        fig.tight_layout()

        # plt.show()
        fig.savefig(join(RESULTS_DIR, 'final_timestep.pdf'))

    return {'res': G, 'esmt': esmt_solver.G}, \
           {'res': G.size(weight="length"), 'esmt': esmt_solver.length, 'star': star_length}, \
           {'res': num_connectors, 'esmt': esmt_solver.added_points} 


def plot_single_trial(path, plot=False):
    s, _, _, _ = load_log_with_checks(path, print_result=plot)

    # df = pd.DataFrame({
    #     'Time':                         pd.Series(dtype='int'),
    #     'Teams':                        pd.Series(dtype='int'), 
    #     # 'Workers Per Team':              pd.Series(dtype='int'),
    #     'Network Length':               pd.Series(dtype='float'),
    #     'Network Length (Solver)':      pd.Series(dtype='float'),
    #     'Connectors':                   pd.Series(dtype='int'),
    #     'Steiner Points (Solver)':      pd.Series(dtype='int'),
    #     'Total Team Length':            pd.Series(dtype='float'),
    #     'Total Team Length (Solver)':   pd.Series(dtype='float')       
    # })

    df1 = pd.DataFrame({
        'Time':                         pd.Series(dtype='int'),
        'Network Length':               pd.Series(dtype='float'),  
        'Type':                         pd.Series(dtype='str'),
    })

    df2 = pd.DataFrame({
        'Time':                         pd.Series(dtype='int'),
        'Connectors':                   pd.Series(dtype='int'),
        'Type':                         pd.Series(dtype='str'),    
    })

    df3 = pd.DataFrame({
        'Time':                         pd.Series(dtype='int'),
        'Connectors':                   pd.Series(dtype='int'),    
        'Type':                         pd.Series(dtype='str'),
    })

    for time in range(7,s.totalTime+1):
        # print(f'time {time}')
        graphs, lengths, connectors = get_network_length_and_connectors_at_time(s, time=time, plot=False)
        # print(f'length {lengths["res"]}')
        res_path_length = get_team_distance(graphs["res"])
        esmt_path_length = get_team_distance(graphs["esmt"])
        # ttrnp_path_length = get_team_distance(graphs["2trnp"])

        # print(f'res_path: {res_path_length}')
        # print(f'approx_path: {approx_path_length}')

            # # Add data of the successful trial
            # if final_points > 0 and final_connectivity:

            # # Find the total network length and connectors for the result obtained and the solver output
            # graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            # if not nx.is_connected(graphs['res']):
            #     trial_no_connectivity[scenario] = {
            #         'seed': s.seed,
            #         'data': False
            #     }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            # res_path_length = get_team_distance(graphs["res"])
            # approx_path_length = get_team_distance(graphs["approx"])

        # d = pd.DataFrame({
        #     'Time': [time],
        #     'Teams': [s.numLeaders], 
        #     # 'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
        #     'Network Length': [lengths['res']], 
        #     'Network Length (Solver)': [lengths['approx']],
        #     'Connectors': [connectors['res']],
        #     'Steiner Points (Solver)': [connectors['approx']],
        #     'Total Team Length': [res_path_length],
        #     'Total Team Length (Solver)': [approx_path_length]  
        # })
        # df = pd.concat([df, d], ignore_index=True, axis=0)

        d = pd.DataFrame({'Time': [time], 'Network Length': [lengths['res']], 'Type': ['res']})
        df1 = pd.concat([df1, d], ignore_index=True, axis=0)
        d = pd.DataFrame({'Time': [time], 'Network Length': [lengths['esmt']], 'Type': ['smt']})
        df1 = pd.concat([df1, d], ignore_index=True, axis=0)
        # d = pd.DataFrame({'Time': [time], 'Network Length': [lengths['2trnp']],'Type': ['2trnp']})
        # df1 = pd.concat([df1, d], ignore_index=True, axis=0)

        d = pd.DataFrame({'Time': [time], 'Connectors': [connectors['res']], 'Type': ['res']})
        df2 = pd.concat([df2, d], ignore_index=True, axis=0)
        d = pd.DataFrame({'Time': [time], 'Connectors': [connectors['esmt']], 'Type': ['smt']})
        df2 = pd.concat([df2, d], ignore_index=True, axis=0)
        # d = pd.DataFrame({'Time': [time], 'Connectors': [connectors['2trnp']], 'Type': ['2trnp']})
        # df2 = pd.concat([df2, d], ignore_index=True, axis=0)

        d = pd.DataFrame({'Time': [time], 'Total Team Length': [res_path_length], 'Type': ['res']})
        df3 = pd.concat([df3, d], ignore_index=True, axis=0)
        d = pd.DataFrame({'Time': [time], 'Total Team Length': [esmt_path_length], 'Type': ['esmt']})
        df3 = pd.concat([df3, d], ignore_index=True, axis=0)
        # d = pd.DataFrame({'Time': [time], 'Total Team Length': [ttrnp_path_length], 'Type': ['2trnp']})
        # df3 = pd.concat([df3, d], ignore_index=True, axis=0)

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(20)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(16)

    # Plot data
    fig = plt.figure(figsize=(9, 5))
    axes = fig.gca()
    fig2 = plt.figure(figsize=(9, 5))
    axes2 = fig2.gca()
    fig3 = plt.figure(figsize=(9, 5))
    axes3 = fig3.gca()

    # order_num_team = list(df['Teams'].unique())
    # order_num_team.sort()
    # order_team_size = list(df['Workers Per Team'].unique())
    # order_team_size.sort()

    # Get 

    sns.lineplot(data=df1, ax=axes, x='Time', y='Network Length', hue='Type', palette='Set2')

    sns.lineplot(data=df2, ax=axes2, x='Time', y='Connectors', hue='Type', palette='Set2')

    sns.lineplot(data=df3, ax=axes3, x='Time', y='Total Team Length', hue='Type', palette='Set2')

    # X = np.repeat(np.atleast_2d(np.arange(len(team_num_dirs))),len(team_num_dirs),axis=0) + np.array([[-.3],[-.1],[.1],[.3]])
    # X = np.repeat(np.atleast_2d(np.arange(len(team_num_dirs))),len(team_num_dirs),axis=0) + np.array([[0]])
    # X = X.flatten()
    # X.sort()
    # approx_length_means = []
    # approx_steiner_means = []
    # approx_team_length_means = []
    # for team in order_num_team:
    #     for size in order_team_size:
    #         df_team = df[df['Teams'] == team]
    #         df_team_and_size = df_team[df_team['Workers Per Team'] == size]
    #         print(f'{team} --- {size}')
    #         print(f'Mean length:\t\t{df_team_and_size["Network Length"].mean()}\tSolver:\t{df_team_and_size["Network Length (Solver)"].mean()}')
    #         print(f'Mean connectors:\t{df_team_and_size["Connectors"].mean()}\tSolver:\t{df_team_and_size["Steiner Points (Solver)"].mean()}')
    #         print(f'Mean team distance:\t\t{df_team_and_size["Total Team Length"].mean()}\tSolver:\t{df_team_and_size["Total Team Length (Solver)"].mean()}')
    #         approx_length_means.append(df_team_and_size["Network Length (Solver)"].mean())
    #         approx_steiner_means.append(df_team_and_size["Steiner Points (Solver)"].mean())
    #         approx_team_length_means.append(df_team_and_size["Total Team Length (Solver)"].mean())

    # print(X)
    # print(approx_length_means)

    # axes.plot(X, approx_length_means, 'r<', zorder=4) # Network Length
    # axes2.plot(X, approx_steiner_means, 'r<', zorder=4)  # Connectors
    # axes3.plot(X, approx_team_length_means, 'r<', zorder=4) # Total Team Length

    # colors = ['.25', '.25']
    # sns.stripplot(data=df, ax=axes, x='Teams', y='Network Length', order=[2], hue='Workers Per Team', hue_order=[6, 12, 18, 24], size=6, palette=colors, linewidth=0, dodge=True, jitter=False)
    
    axes.set_xlabel('Timestep', fontproperties=font)
    axes.set_ylabel('Network Length (m)', fontproperties=font)

    axes2.set_xlabel('Timestep', fontproperties=font)
    axes2.set_ylabel('Connectors', fontproperties=font)

    axes3.set_xlabel('Timestep', fontproperties=font)
    axes3.set_ylabel('Total Team Path (m)', fontproperties=font)

    # border
    axes.spines['top'].set_visible(False)
    axes.spines['right'].set_visible(False)
    axes.spines['bottom'].set_linewidth(1)
    axes.tick_params(width=1)

    axes2.spines['top'].set_visible(False)
    axes2.spines['right'].set_visible(False)
    axes2.spines['bottom'].set_linewidth(1)
    axes2.tick_params(width=1)

    axes3.spines['top'].set_visible(False)
    axes3.spines['right'].set_visible(False)
    axes3.spines['bottom'].set_linewidth(1)
    axes3.tick_params(width=1)

    # axes.set_ylim([0,4])
    
    fig.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()

    plt.show()


def plot_single_trial_at_time(path, time=0, plot=False, final_timestep=False, real_robot=False):
    s, _, _, _ = load_log_with_checks(path, print_result=True)
    if final_timestep:
        time = s.totalTime
    print('Plotting at timestep = ', time)
    graphs, lengths, connectors = get_network_length_and_connectors_at_time(s, time=time, plot=plot, real_robot=real_robot)
    # res_path_length = get_team_distance(graphs['res'])
    # esmt_path_length = get_team_distance(graphs['esmt'])
    # star_path_length = get_team_distance(graphs['star'])
    print(f'res_path: {lengths["res"]}')
    print(f'esmt_path: {lengths["esmt"]}')
    print(f'star_path: {lengths["star"]}')


def plot_all_trials_congestion(var, max_t=0):
    
    df = pd.DataFrame({
        'Time':             pd.Series(dtype='float'), 
        'Delay':            pd.Series(dtype='float'),
        'Travelers':        pd.Series(dtype='int'),
        'Average Speed':    pd.Series(dtype='float'),  
        'Minimum Speed':    pd.Series(dtype='float'),
        'Total Distance':   pd.Series(dtype='float'),
        'Robots Collided':  pd.Series(dtype='int'),
    })
    
    # Get trial names
    variation_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    variation_dirs.sort()

    num_failed_runs = {}
    
    max_time = max_t

    for variation in variation_dirs:
        if variation == var:
            trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
            trial_dirs.sort()

            trial_no_points = {}
            trial_no_connectivity = {}
            trial_no_team_connectivity = {}
            trial_timeout = {}

            start_time = time.time()

            count = 0
            # for scenario in trial_dirs:
            while count < len(trial_dirs):
                # if count >= len(trial_dirs):
                #     break

                scenario = trial_dirs[count]
                delay = int(scenario.split('_')[3][:-1]) # Get send delay

                start_time_single = time.time()

                # Check if the trial was successful
                s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, variation, scenario))
                if final_points == 0:
                    trial_no_points[scenario] = {
                        'seed': s.seed,
                        'data': final_points
                    }

                if final_team_connectivity:
                    trial_no_team_connectivity[scenario] = {
                        'seed': s.seed,
                        'data': final_team_connectivity
                    }

                # Find the total network length and connectors for the result obtained and the solver output
                graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
                if not nx.is_connected(graphs['res']):
                    trial_no_connectivity[scenario] = {
                        'seed': s.seed,
                        'data': False
                    }

                if s.totalTime == 6000:
                    trial_timeout[scenario] = {
                        'seed': s.seed,
                        'data': True
                    }

                # Find the number of Travelers and the average speed of travelers at every timestep
                data, avg_travel_time = get_traveler_info(s)

                # print(f'AVG TRAVEL TIME: {avg_travel_time}')

                # Add data of the successful trial
                if not scenario in trial_no_connectivity and not scenario in trial_timeout:
                    
                    last_time = 0
                    last_entry = None
                    count_entry = 0
                    
                    for t, log in data.items():
                        d = pd.DataFrame({
                            'Time': [int(t/10)], 
                            'Delay': [delay/10], 
                            'Travelers': [log['num_traveler']], 
                            'Average Speed': [log['avg_speed'] if log['avg_speed'] != None else np.nan], # Replace None with np.nan
                            'Minimum Speed': [log['min_speed'] if log['min_speed'] != None else np.nan],
                            'Total Distance': [log['total_dist']],
                            'Robots Collided': [s.robotsCollided],
                        })
                        last_entry = d
                        last_time = t
                        count_entry += 1
                        df = pd.concat([df, d], ignore_index=True, axis=0)
                        
                        # # FOR FINDING THE MAX_TIME OF ALL TRIALS
                        # if t > max_time:
                        #     max_time = t
                                                    
                    # Repeat last timestep entry
                    # count_t = last_time
                    count_t = int(last_time/10)*10
                    while count_t < max_time:
                        count_t += 10
                        last_entry['Time'] = count_t/10
                        df = pd.concat([df, d], ignore_index=True, axis=0)
                        count_entry += 1
                        
                    # print(f'lines: {count_entry}')
                    
                else:
                    # Record failed trial
                    delay_val = int(scenario.split('_')[3][:-1])
                    trial_num = int(scenario.split('_')[4])
                    if not delay_val in num_failed_runs:
                        num_failed_runs[delay_val] = set()
                    num_failed_runs[delay_val].add(trial_num)

                duration_single = round(time.time() - start_time_single, 3)
                duration_total = round(time.time() - start_time, 3)
                print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

                # DEBUG: For limiting the number of data to plot
                count += 1
                num_to_use = 50
                if (count % 10) % num_to_use == 0:
                    count += 50 - num_to_use
                    # print(count)
                    # break


            print(f'### Lost global connectivity ###')
            for scenario, trial in trial_no_connectivity.items():
                print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
            print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

            print(f'### Timeout ###')
            for scenario, trial in trial_timeout.items():
                print(f'      scenario: {scenario}, seed: {trial["seed"]}, timeout: {trial["data"]}')
            print(f'    Trials that timed out: ({len(trial_timeout)}/{len(trial_dirs)})')

            # print(df['Delay'].unique())
            
    print(f'Failed Runs: {num_failed_runs}')
    print(f'max timestep: {max_time}')

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    # Plot data
    fig1 = plt.figure(figsize=(5, 4))
    axes1 = fig1.gca()
    # fig2 = plt.figure(figsize=(9, 5))
    # axes2 = fig2.gca()
    # fig3 = plt.figure(figsize=(9, 5))
    # axes3 = fig3.gca()
    # fig4 = plt.figure(figsize=(9, 5))
    # axes4 = fig4.gca()
    fig5 = plt.figure(figsize=(5, 4))
    axes5 = fig5.gca()
    axes = [axes1, axes5]

    print(df)

    ### LINE PLOT
    hue_order = np.sort(df['Delay'].unique())
    print(f'hue_order {hue_order}')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Travelers', hue='Delay', hue_order=hue_order, legend='full')
    # sns.lineplot(data=df, ax=axes2, x='Time', y='Average Speed', hue='Delay', hue_order=hue_order)
    # sns.lineplot(data=df, ax=axes3, x='Time', y='Minimum Speed', hue='Delay', hue_order=hue_order)
    # sns.lineplot(data=df, ax=axes4, x='Time', y='Total Distance', hue='Delay', hue_order=hue_order)

    sns.boxplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, palette='Set2')
    # colors = ['.25', '.25']
    # sns.stripplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, size=6, palette=colors, linewidth=0, dodge=True, jitter=False)

    axes1.set_xlabel('Time (s)', fontproperties=font)
    axes1.set_ylabel('Travellers', fontproperties=font)

    # legend

    # axes1.legend([str(int(label)) for label in hue_order], bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    handles, labels = axes1.get_legend_handles_labels()
    # axes1.legend(handles=handles[1:], labels=[str(int(label)) for label in hue_order], title='Delay (s)')
    axes1.legend(handles=handles, labels=[str(int(float(label))) for label in labels], title='Delay (s)')
    axes1.set_ylim([0,40+1])


    max_val = df['Robots Collided'].max()
    # axes5.set_yticks(np.arange(0, max_val+2, 2))
    axes5.set_xticks(range(len(labels)), labels=[str(int(float(label))) for label in labels])
    axes5.set_xlabel('Delay (s)', fontproperties=font)
    axes5.set_ylabel('Robots Collided', fontproperties=font)
    axes5.set_ylim([0,26+3])
    print(labels)
    # axes5.set_xticks(range(len(labels)))

    # Define the text to be displayed above each box plot
    text_values = []
    for label in labels:
        delay_val = int(float(label)) * 10
        if delay_val in num_failed_runs:
            text_values.append(f'x{len(num_failed_runs[delay_val])}')
        else:
            text_values.append('x0')

    print(text_values)

    # Calculate the x-coordinate for each text based on the number of box plots (data points)
    num_boxes = len(text_values)
    text_positions = range(num_boxes)

    # Add text above each box plot
    for i, t in zip(text_positions, text_values):
        axes5.text(i, 26 + 2, t, ha='center', color='red', fontproperties=font2)

    for ax in axes:
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

    fig1.tight_layout()
    fig5.tight_layout()

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, f'congestion_{var}_travellers.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'congestion_{var}_collisions.pdf'))  


def get_traveler_info(s):
    """Find the number of travelers, the average and minimum speed of travelers in each timestep"""

    data = {}

    prev_traveler_pos = {}
    traveler_time = {}

    sim_step = 0.1
    ex_timestep = 1 # Duration between timesteps being read (s)
    total_dist = 0

    for t in range(1,s.totalTime+1,int(ex_timestep/sim_step)):
        
        d = {}
        d['num_traveler'] = 0
        d['avg_speed'] = None
        d['min_speed'] = None
        t_dist = 0
        min_dist = None

        for robot in s[t]['log'].robots:
            if robot.state == time_step_pb2.Robot.TRAVELER:
                d['num_traveler'] += 1
                current_pos = (robot.position.x, robot.position.y)

                # Calculate the distance traveled from the previous timestep
                if robot.name in prev_traveler_pos:
                    prev_pos = (prev_traveler_pos[robot.name][0], prev_traveler_pos[robot.name][1])
                    dist = math.dist(prev_pos, current_pos)
                    t_dist += dist

                    if min_dist == None or dist < min_dist:
                        min_dist = dist

                # Add/Update traveler position
                prev_traveler_pos[robot.name] = current_pos

                # Add/Update time as traveler
                if robot.name in traveler_time:
                    traveler_time[robot.name] += ex_timestep/sim_step
                else:
                    traveler_time[robot.name] = 0

        # Calculate the average speed of travelers during this timestep
        if d['num_traveler'] > 0:
            d['avg_speed'] = t_dist / ex_timestep / d['num_traveler']
        if min_dist != None:
            d['min_speed'] = min_dist / ex_timestep

        total_dist += t_dist
        d['total_dist'] = total_dist

        data[t] = d

    # If the final timestep is not added, add it
    if not s.totalTime in data:
        t = s.totalTime
        d = {}
        d['num_traveler'] = 0
        d['avg_speed'] = None
        d['min_speed'] = None
        t_dist = 0
        min_dist = None

        for robot in s[t]['log'].robots:
            if robot.state == time_step_pb2.Robot.TRAVELER:
                d['num_traveler'] += 1
                current_pos = (robot.position.x, robot.position.y)

                # Calculate the distance traveled from the previous timestep
                if robot.name in prev_traveler_pos:
                    prev_pos = (prev_traveler_pos[robot.name][0], prev_traveler_pos[robot.name][1])
                    dist = math.dist(prev_pos, current_pos)
                    t_dist += dist

                    if min_dist == None or dist < min_dist:
                        min_dist = dist

                # Add/Update traveler position
                prev_traveler_pos[robot.name] = current_pos

                # Add/Update time as traveler
                if robot.name in traveler_time:
                    traveler_time[robot.name] += t - max(list(data.keys()))
                else:
                    traveler_time[robot.name] = 0

        # Calculate the average speed of travelers during this timestep
        if d['num_traveler'] > 0:
            d['avg_speed'] = t_dist / ex_timestep / d['num_traveler']
        if min_dist != None:
            d['min_speed'] = min_dist / ex_timestep

        total_dist += t_dist
        d['total_dist'] = total_dist
        
        data[t] = d

    # Average duration a robot spent as a traveler (for those which did travel)
    total_travel_time = sum(list(traveler_time.values()))
    avg_travel_time = total_travel_time / len(traveler_time)

    return data, avg_travel_time


def plot_network_length():
    
    df = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        'Workers Per Team':                  pd.Series(dtype='int'),
        'Network Length':                   pd.Series(dtype='float'),
        'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        'Connectors':                       pd.Series(dtype='int'),
        'Connectors (ESMT)':   pd.Series(dtype='float'),
        # 'Steiner Points (2tRNP)':           pd.Series(dtype='int'),
        # 'Total Team Length':                pd.Series(dtype='float'),
        # 'Total Team Length (CDWX-2008)':    pd.Series(dtype='float'),       
        # 'Total Team Length (CDWX-2008-EX)': pd.Series(dtype='float'),
        # 'Total Team Length (2tRNP)':        pd.Series(dtype='float')       
    })

    df_compare = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        # 'Workers Per Team':                  pd.Series(dtype='int'),
        'Solution':                            pd.Series(dtype='str'),
        'Network Length':                   pd.Series(dtype='float'),
        'ID':                               pd.Series(dtype='int'),
        # 'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        # 'Connectors':                       pd.Series(dtype='int'),
        # 'Connectors (ESMT)':   pd.Series(dtype='float'),
    })

    # Get trial names
    team_num_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    team_num_dirs.sort()

    print(team_num_dirs)

    for team_num in team_num_dirs:

        # if team_num[-1] != 'T':
        #     continue # don't include 'star'

        if team_num[-1] != 'T':
            solution = 'Our approach (without branch repositioning)'
        else :
            solution = 'Our approach'

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, team_num)) if isdir(join(RESULTS_DIR, team_num, f))]
        trial_dirs.sort()

        # print(trial_dirs)

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0

        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distance(graphs["res"])
            esmt_path_length = get_team_distance(graphs["esmt"])
            # ttrnp_path_length = get_team_distance(graphs["2trnp"])

            # Add data of the successful trial
            # if not scenario in trial_no_points and not scenario in trial_no_connectivity:
            if not scenario in trial_no_connectivity:

                d = pd.DataFrame({
                    'Teams': [s.numLeaders], 
                    'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
                    'Network Length': [lengths['res']], 
                    'Network Length (ESMT)': [lengths['esmt']],
                    'Network Length (Star)': [lengths['star']],
                    # 'Network Length (2tRNP)': [lengths['2trnp']],
                    'Connectors': [connectors['res']],
                    'Connectors (ESMT)': [connectors['esmt']],
                    # 'Connectors (2tRNP)': [connectors['2trnp']],
                    'Total Team Length': [res_path_length],
                    'Total Team Length (ESMT)': [esmt_path_length],
                    # 'Total Team Length (2tRNP)': [ttrnp_path_length]  
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

                if(s.numLeaders > 2):
                    # # Sum
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Central Solver'], 'Network Length': [lengths['esmt']]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Our Approach'], 'Network Length': [lengths['res']]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Star-like Topology'], 'Network Length': [lengths['star']]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                    # Average
                    if solution == 'Our approach':
                        d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Steiner tree'], 'Network Length': [lengths['esmt']/s.numLeaders], 'ID': [count]})
                        df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                        d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': [solution], 'Network Length': [lengths['res']/s.numLeaders], 'ID': [count]})
                        df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                        d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Optimal starlike tree'], 'Network Length': [lengths['star']/s.numLeaders], 'ID': [count]})
                        df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)  
                    else:
                        d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': [solution], 'Network Length': [lengths['res']/s.numLeaders], 'ID': [count]})
                        df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
            else:
                # change output color of print
                
                print(Fore.LIGHTRED_EX + 'skipping... ' + scenario)

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            # num_to_use = 50
            # if (count % 10) % num_to_use == 0:
            #     count += 50 - num_to_use
            #     # print(count)
            #     # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig1 = plt.figure(figsize=(6, 6))
    axes1 = fig1.gca()
    fig2 = plt.figure(figsize=(6, 6))
    axes2 = fig2.gca()
    fig3 = plt.figure(figsize=(6, 6))
    axes3 = fig3.gca()
    fig4 = plt.figure(figsize=(5.5, 6.5))
    axes4 = fig4.gca()

    axes = [axes1, axes2, axes3, axes4]

    order_num_team = list(df['Teams'].unique())
    order_num_team.sort()
    order_team_size = list(df['Workers Per Team'].unique())
    order_team_size.sort()

    ### SCATTER PLOT
    x = list(range(16))
    axes1.plot(x,x, color='black', linestyle='dashed')
    frequency = df.groupby(['Connectors (ESMT)', 'Connectors']).size().reset_index(name='Frequency')
    sns.scatterplot(
        data=frequency, ax=axes1,
        x='Connectors (ESMT)', y='Connectors', hue='Frequency', size='Frequency', sizes=(250,1000),
        # edgecolor='gray',
        legend=False
    )
    
    for i, row in frequency.iterrows():
        count = int(row['Frequency'])
        if count > 10:
            color = 'white'
        else:
            color = 'black'
        axes1.text(row['Connectors (ESMT)'], row['Connectors'], str(count), ha='center', va='center', color=color)

    axes2.plot(x,x, color='black', linestyle='dashed')
    sns.scatterplot(
        data=df, ax=axes2,
        x='Network Length (ESMT)', y='Network Length', hue='Teams', palette='Set1'
    )

    print(df)

    axes3.plot(x,x, color='black', linestyle='dashed')
    sns.scatterplot(
        data=df, ax=axes3,
        x='Network Length (ESMT)', y='Network Length (Star)', hue='Teams', palette='Set1'
    )

    # sns.boxplot(data=df, ax=axes4, x='Teams', y='Network Length', hue='Network', palette='Set1')
    # Add an extra column to the dataframe for the hue where it is either 'Network Length', 'Network Length (ESMT)', and 'Network Length (Star)'
    # df['Network'] = 'Network Length'
    # df2 = df.copy()
    # df2['Network'] = 'Network Length (ESMT)'
    # df3 = df.copy()
    # df3['Network'] = 'Network Length (Star)'
    # df4 = pd.concat([df, df2, df3], ignore_index=True, axis=0)
    # print(df4)

    # calculate the mean network length for each team and solution combination
    # df_compare = df4.groupby(['Teams', 'Network']).mean().reset_index()
    # print(df_compare)

    g = sns.boxplot(data=df_compare, ax=axes4, x='Teams', y='Network Length', hue='Solution', palette='Set2')
    
    # https://stackoverflow.com/questions/72656861/how-to-add-hatches-to-boxplots-with-sns-boxplot-or-sns-catplot
    # hatches must equal the number of hues (4 in this case)
    hatches = ['//', '..', 'xx', '/']

    # iterate through each subplot / Facet
    for ax in [g.axes]:

        # select the correct patches
        patches = [patch for patch in ax.patches if type(patch) == PathPatch]
        # the number of patches should be evenly divisible by the number of hatches
        print('patches:', len(patches))
        print('hatches:', len(hatches))
        # create a list ['//', '//', '//', '//', '..', '..', '..', '..', 'xx', 'xx', 'xx', 'xx'] using a loop
        # multiply the hatches by the number of boxes in the plot

        h = int(len(patches) / len(hatches))
        print('h:',h)
        result = [hatch for hatch in hatches for _ in range(h)]
        print(result)
        print('h:', h)
        # iterate through the patches for each subplot
        for patch, hatch in zip(patches, result):
            patch.set_hatch(hatch)
            # fc = patch.get_facecolor()
            patch.set_edgecolor('black')

            # change median line to also be black
            for line in ax.lines:
                line.set_color('black')

    # place the legend on the top of the plot and place each item horizontally
    legend = axes4.legend(loc='upper center', bbox_to_anchor=(0.5, 1.4), ncol=1)
    # change the font of the text in the legend
    for text in legend.get_texts():
        text.set_fontsize(16)
    
    # Make the legend patches also match the box plot appearances
    for lp, hatch in zip(legend.get_patches(), hatches):
        lp.set_hatch(hatch)
        lp.set_edgecolor('black')

    # # stats annotation: resuires seaborn <= 0.12.2 !!
    # pairs = [[(3,'Our approach'), (3, 'Steiner tree')]]
    # annotator = Annotator(axes4, pairs, data=df_compare, x='Teams', y='Network Length', hue='Solution', order=[3,4,5,6])
    # annotator.configure(test='Wilcoxon', text_format='star', loc='inside')
    # annotator.apply_and_annotate()

        # stats annotation: resuires seaborn <= 0.12.2 !!
    # pairs = [('Our approach', 'Starlike topology')]
    # annotator = Annotator(axes4, pairs, data=df_compare, x='Solution', y='Network Length', order=['Our approach', 'Starlike topology'])
    # annotator.configure(test='Wilcoxon', text_format='star', loc='inside')
    # annotator.apply_and_annotate()

    # run mann whitney test to check whether the results of Network and Network (star) are statistically significant

    # Change the color of the mean line to Crimson

    # for line in axes4.get_lines():
    #     # check if the line is a dash
    #     if line.get_linestyle() == '--':
    #         line.set_color('Crimson')
    
    # axes4.plot([], [], '--', linewidth=1, color='Crimson', label='mean')
    # axes4.legend()

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(18)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(16)

    font3 = FontProperties()
    # font3.set_family('serif')
    # font3.set_name('Times New Roman')
    font3.set_size(14)

    font4 = FontProperties()
    # font4.set_family('serif')
    # font4.set_name('Times New Roman')
    font4.set_size(14)

    # Set axis labels
    # axes.set_xlabel("Average waiting time per task (s)")
    axes1.set_xlabel("added points: ESMT, n", fontproperties=font)
    axes1.set_ylabel("connectors: simulation, n", fontproperties=font)
    axes2.set_xlabel("network length: Steiner tree (m)", fontproperties=font)
    axes2.set_ylabel("network length: Simulation (m)", fontproperties=font)
    axes3.set_xlabel("network length: Steiner tree (m)", fontproperties=font)
    axes3.set_ylabel("network length: Star topology (m)", fontproperties=font)
    axes4.set_ylabel(r'Network length / $n$ (m)', fontproperties=font)
    axes4.set_xlabel(r'Number of locations $n$', fontproperties=font)
    # for key, ax in axd9.items():
    #     if key != 'r':
    #         ax.set_xlabel("Teams", fontproperties=font)
    #         ax.set_ylabel("Workers Per Team", fontproperties=font)

    #     for label in ax.get_xticklabels():
    #         label.set_fontproperties(font2)
    #     for label in ax.get_yticklabels():
    #         label.set_fontproperties(font2)

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

    # border and ticks
    max_val = max(df['Connectors'].max(),df['Connectors (ESMT)'].max())
    axes1.set_xticks(np.arange(0, max_val+2, 1))
    axes1.set_yticks(np.arange(0, max_val+2, 1))
    axes1.set_xlim([0,max_val+2])
    axes1.set_ylim([0,max_val+2])

    max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    axes2.set_xticks(np.arange(0, max_val+2, 1))
    axes2.set_yticks(np.arange(0, max_val+2, 1))
    axes2.set_xlim([0,max_val+2])
    axes2.set_ylim([0,max_val+2])

    # max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    axes3.set_xticks(np.arange(0, max_val+2, 1))
    axes3.set_yticks(np.arange(0, max_val+2, 1))
    axes3.set_xlim([0,max_val+2])
    axes3.set_ylim([0,max_val+2])

    # print(df.to_string())
    print(f'Mean Connectors: {df["Connectors"].mean()}')
    print(f'Mean Connectors (ESMT): {df["Connectors (ESMT)"].mean()}')
    print(f'Mean Network Length: {df["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT): {df["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star): {df["Network Length (Star)"].mean()}')

    # print mean network length for 2 teams
    print(f'Mean Network Length (2 teams): {df[df["Teams"] == 2]["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT, 2 teams): {df[df["Teams"] == 2]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 2 teams): {df[df["Teams"] == 2]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (3 teams): {df[df["Teams"] == 3]["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT, 3 teams): {df[df["Teams"] == 3]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 3 teams): {df[df["Teams"] == 3]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (4 teams): {df[df["Teams"] == 4]["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT, 4 teams): {df[df["Teams"] == 4]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 4 teams): {df[df["Teams"] == 4]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (5 teams): {df[df["Teams"] == 5]["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT, 5 teams): {df[df["Teams"] == 5]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 5 teams): {df[df["Teams"] == 5]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (6 teams): {df[df["Teams"] == 6]["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT, 6 teams): {df[df["Teams"] == 6]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 6 teams): {df[df["Teams"] == 6]["Network Length (Star)"].mean()}')

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (2tRNP)', 'w = 12', 'w = 12 (2tRNP)', 'w = 18', 'w = 18 (2tRNP)', 'w = 24', 'w = 24 (2tRNP)']
    # legend_labels2 = ['t = 2', 't = 2 (2tRNP)', 't = 3', 't = 3 (2tRNP)', 't = 4', 't = 4 (2tRNP)', 't = 5', 't = 5 (2tRNP)', 't = 6', 't = 6 (2tRNP)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes2.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    # set font properties of colorbar label
    # cbar7 = axes7.collections[0].colorbar
    # cbar7.set_label("Ratio", fontdict={'size': 14})
    # cbar7.ax.tick_params(labelsize=14)
    # cbar8 = axes8.collections[0].colorbar
    # cbar8.set_label("Ratio", fontdict={'size': 14})
    # cbar8.ax.tick_params(labelsize=14)

    # cbar8 = axd9['r'].collections[0].colorbar
    # axd9['r'].set_ylabel("Ratio", fontdict={'size': 14})
    # axd9['r'].tick_params(labelsize=14)

    # for key, ax in axd9.items():
    #     if key == 'r':
    #         ax.spines['top'].set_visible(True)
    #         ax.spines['right'].set_visible(True)
    #     else:
    #         ax.spines['top'].set_visible(False)
    #         ax.spines['right'].set_visible(False)
    #     ax.spines['left'].set_visible(True)
    #     ax.spines['bottom'].set_visible(True)
    #     ax.spines['left'].set_linewidth(1)
    #     ax.spines['bottom'].set_linewidth(1)
    #     ax.tick_params(width=1)

    # # set axis4 ymin and ymax
    # axes4.set_ylim([0.5940869831534189, 1.7858354709342863])

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    # axes1.axis('equal')
    # axes2.axis('equal')

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'connectors_ex0.pdf'))
    fig2.savefig(join(RESULTS_DIR, 'network_length_ex0.pdf'))  
    fig3.savefig(join(RESULTS_DIR, 'network_length_ex0_star.pdf'))  
    fig4.savefig(join(RESULTS_DIR, 'network_length_comparison.pdf'))  

    df_compare.to_csv(join(RESULTS_DIR, 'network_length.csv'), index=False)


def plot_network_length_star():
    
    df = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        'Workers Per Team':                  pd.Series(dtype='int'),
        'Network Length':                   pd.Series(dtype='float'),
        'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        'Connectors':                       pd.Series(dtype='int'),
        'Connectors (ESMT)':   pd.Series(dtype='float'),
        # 'Steiner Points (2tRNP)':           pd.Series(dtype='int'),
        # 'Total Team Length':                pd.Series(dtype='float'),
        # 'Total Team Length (CDWX-2008)':    pd.Series(dtype='float'),       
        # 'Total Team Length (CDWX-2008-EX)': pd.Series(dtype='float'),
        # 'Total Team Length (2tRNP)':        pd.Series(dtype='float')       
    })

    df_compare = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        # 'Workers Per Team':                  pd.Series(dtype='int'),
        'Solution':                            pd.Series(dtype='str'),
        'Network Length':                   pd.Series(dtype='float'),
        'ID':                               pd.Series(dtype='int'),
        # 'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        # 'Connectors':                       pd.Series(dtype='int'),
        # 'Connectors (ESMT)':   pd.Series(dtype='float'),
    })

    # Get trial names
    team_num_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    team_num_dirs.sort()

    print(team_num_dirs)

    for team_num in team_num_dirs:

        if team_num[-1] == 'T':
            continue # only include 'star'

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, team_num)) if isdir(join(RESULTS_DIR, team_num, f))]
        trial_dirs.sort()

        # print(trial_dirs)

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0

        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distance(graphs["res"])
            esmt_path_length = get_team_distance(graphs["esmt"])
            # ttrnp_path_length = get_team_distance(graphs["2trnp"])

            # Add data of the successful trial
            # if not scenario in trial_no_points and not scenario in trial_no_connectivity:
            if not scenario in trial_no_connectivity:

                d = pd.DataFrame({
                    'Teams': [s.numLeaders], 
                    'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
                    'Network Length': [lengths['res']], 
                    'Network Length (ESMT)': [lengths['esmt']],
                    'Network Length (Star)': [lengths['star']],
                    # 'Network Length (2tRNP)': [lengths['2trnp']],
                    'Connectors': [connectors['res']],
                    'Connectors (ESMT)': [connectors['esmt']],
                    # 'Connectors (2tRNP)': [connectors['2trnp']],
                    'Total Team Length': [res_path_length],
                    'Total Team Length (ESMT)': [esmt_path_length],
                    # 'Total Team Length (2tRNP)': [ttrnp_path_length]  
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

                if(s.numLeaders > 2):
                    # # Sum
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Central Solver'], 'Network Length': [lengths['esmt']]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Our Approach'], 'Network Length': [lengths['res']]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Star-like Topology'], 'Network Length': [lengths['star']]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                    # Average
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Steiner tree'], 'Network Length': [lengths['esmt']/s.numLeaders], 'ID': [count]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                    d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Optimal starlike topology'], 'Network Length': [lengths['star']/s.numLeaders], 'ID': [count]})
                    df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)  
                    d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Our approach (without branch repositioning)'], 'Network Length': [lengths['res']/s.numLeaders], 'ID': [count]})
                    df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

            else:
                # change output color of print
                
                print(Fore.LIGHTRED_EX + 'skipping... ' + scenario)

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            # num_to_use = 50
            # if (count % 10) % num_to_use == 0:
            #     count += 50 - num_to_use
            #     # print(count)
            #     # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig1 = plt.figure(figsize=(6, 6))
    axes1 = fig1.gca()
    fig2 = plt.figure(figsize=(6, 6))
    axes2 = fig2.gca()
    fig3 = plt.figure(figsize=(6, 6))
    axes3 = fig3.gca()
    fig4 = plt.figure(figsize=(4.6, 6))
    axes4 = fig4.gca()

    axes = [axes1, axes2, axes3, axes4]

    order_num_team = list(df['Teams'].unique())
    order_num_team.sort()
    order_team_size = list(df['Workers Per Team'].unique())
    order_team_size.sort()

    ### SCATTER PLOT
    x = list(range(16))
    axes1.plot(x,x, color='black', linestyle='dashed')
    frequency = df.groupby(['Connectors (ESMT)', 'Connectors']).size().reset_index(name='Frequency')
    sns.scatterplot(
        data=frequency, ax=axes1,
        x='Connectors (ESMT)', y='Connectors', hue='Frequency', size='Frequency', sizes=(250,1000),
        # edgecolor='gray',
        legend=False
    )
    
    for i, row in frequency.iterrows():
        count = int(row['Frequency'])
        if count > 10:
            color = 'white'
        else:
            color = 'black'
        axes1.text(row['Connectors (ESMT)'], row['Connectors'], str(count), ha='center', va='center', color=color)

    axes2.plot(x,x, color='black', linestyle='dashed')
    sns.scatterplot(
        data=df, ax=axes2,
        x='Network Length (ESMT)', y='Network Length', hue='Teams', palette='Set1'
    )

    print(df)

    axes3.plot(x,x, color='black', linestyle='dashed')
    sns.scatterplot(
        data=df, ax=axes3,
        x='Network Length (ESMT)', y='Network Length (Star)', hue='Teams', palette='Set1'
    )

    # sns.boxplot(data=df, ax=axes4, x='Teams', y='Network Length', hue='Network', palette='Set1')
    # Add an extra column to the dataframe for the hue where it is either 'Network Length', 'Network Length (ESMT)', and 'Network Length (Star)'
    # df['Network'] = 'Network Length'
    # df2 = df.copy()
    # df2['Network'] = 'Network Length (ESMT)'
    # df3 = df.copy()
    # df3['Network'] = 'Network Length (Star)'
    # df4 = pd.concat([df, df2, df3], ignore_index=True, axis=0)
    # print(df4)

    # calculate the mean network length for each team and solution combination
    # df_compare = df4.groupby(['Teams', 'Network']).mean().reset_index()
    # print(df_compare)

    # g = sns.boxplot(data=df_compare, ax=axes4, x='Teams', y='Network Length', hue='Solution', palette='Set2')
    g = sns.boxplot(data=df_compare, ax=axes4, x='Teams', y='Network Length', hue='Solution', palette=['#8da0cb','#e78ac3'])

    # https://stackoverflow.com/questions/72656861/how-to-add-hatches-to-boxplots-with-sns-boxplot-or-sns-catplot
    # hatches must equal the number of hues (2 in this case)
    hatches = ['xx','/']

    # iterate through each subplot / Facet
    for ax in [g.axes]:

        # select the correct patches
        patches = [patch for patch in ax.patches if type(patch) == PathPatch]
        # the number of patches should be evenly divisible by the number of hatches
        print('patches:', len(patches))
        print('hatches:', len(hatches))
        # create a list ['//', '//', '//', '//', '..', '..', '..', '..', 'xx', 'xx', 'xx', 'xx'] using a loop
        # multiply the hatches by the number of boxes in the plot

        h = int(len(patches) / len(hatches))
        print('h:',h)
        result = [hatch for hatch in hatches for _ in range(h)]
        print(result)
        print('h:', h)
        # iterate through the patches for each subplot
        for patch, hatch in zip(patches, result):
            patch.set_hatch(hatch)
            # fc = patch.get_facecolor()
            patch.set_edgecolor('black')

            # change median line to also be black
            for line in ax.lines:
                line.set_color('black')

    # place the legend on the top of the plot and place each item horizontally
    legend = axes4.legend(loc='upper center', bbox_to_anchor=(0.32, 1.25), ncol=1)
    # change the font of the text in the legend
    for text in legend.get_texts():
        text.set_fontsize(16)
    
    # Make the legend patches also match the box plot appearances
    for lp, hatch in zip(legend.get_patches(), hatches):
        lp.set_hatch(hatch)
        lp.set_edgecolor('black')

    # make new line in legend text
    for text in legend.get_texts():
        text.set_text(text.get_text() + '\n')
        

    # # stats annotation: resuires seaborn <= 0.12.2 !!
    # pairs = [[(3,'Our approach'), (3, 'Steiner tree')]]
    # annotator = Annotator(axes4, pairs, data=df_compare, x='Teams', y='Network Length', hue='Solution', order=[3,4,5,6])
    # annotator.configure(test='Wilcoxon', text_format='star', loc='inside')
    # annotator.apply_and_annotate()

        # stats annotation: resuires seaborn <= 0.12.2 !!
    # pairs = [('Our approach', 'Starlike topology')]
    # annotator = Annotator(axes4, pairs, data=df_compare, x='Solution', y='Network Length', order=['Our approach', 'Starlike topology'])
    # annotator.configure(test='Wilcoxon', text_format='star', loc='inside')
    # annotator.apply_and_annotate()

    # run mann whitney test to check whether the results of Network and Network (star) are statistically significant

    # Change the color of the mean line to Crimson

    # for line in axes4.get_lines():
    #     # check if the line is a dash
    #     if line.get_linestyle() == '--':
    #         line.set_color('Crimson')
    
    # axes4.plot([], [], '--', linewidth=1, color='Crimson', label='mean')
    # axes4.legend()

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(18)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(16)

    font3 = FontProperties()
    # font3.set_family('serif')
    # font3.set_name('Times New Roman')
    font3.set_size(14)

    font4 = FontProperties()
    # font4.set_family('serif')
    # font4.set_name('Times New Roman')
    font4.set_size(14)

    # Set axis labels
    # axes.set_xlabel("Average waiting time per task (s)")
    axes1.set_xlabel("added points: ESMT, n", fontproperties=font)
    axes1.set_ylabel("connectors: simulation, n", fontproperties=font)
    axes2.set_xlabel("network length: Steiner tree (m)", fontproperties=font)
    axes2.set_ylabel("network length: Simulation (m)", fontproperties=font)
    axes3.set_xlabel("network length: Steiner tree (m)", fontproperties=font)
    axes3.set_ylabel("network length: Star topology (m)", fontproperties=font)
    axes4.set_ylabel(r'Network length / $n$ (m)', fontproperties=font)
    axes4.set_xlabel(r'Number of locations $n$', fontproperties=font)
    # for key, ax in axd9.items():
    #     if key != 'r':
    #         ax.set_xlabel("Teams", fontproperties=font)
    #         ax.set_ylabel("Workers Per Team", fontproperties=font)

    #     for label in ax.get_xticklabels():
    #         label.set_fontproperties(font2)
    #     for label in ax.get_yticklabels():
    #         label.set_fontproperties(font2)

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

    # border and ticks
    max_val = max(df['Connectors'].max(),df['Connectors (ESMT)'].max())
    axes1.set_xticks(np.arange(0, max_val+2, 1))
    axes1.set_yticks(np.arange(0, max_val+2, 1))
    axes1.set_xlim([0,max_val+2])
    axes1.set_ylim([0,max_val+2])

    max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    axes2.set_xticks(np.arange(0, max_val+2, 1))
    axes2.set_yticks(np.arange(0, max_val+2, 1))
    axes2.set_xlim([0,max_val+2])
    axes2.set_ylim([0,max_val+2])

    # max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    axes3.set_xticks(np.arange(0, max_val+2, 1))
    axes3.set_yticks(np.arange(0, max_val+2, 1))
    axes3.set_xlim([0,max_val+2])
    axes3.set_ylim([0,max_val+2])

    # print(df.to_string())
    print(f'Mean Connectors: {df["Connectors"].mean()}')
    print(f'Mean Connectors (ESMT): {df["Connectors (ESMT)"].mean()}')
    print(f'Mean Network Length: {df["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT): {df["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star): {df["Network Length (Star)"].mean()}')

    # print mean network length for 2 teams
    print(f'Mean Network Length (2 teams): {df[df["Teams"] == 2]["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT, 2 teams): {df[df["Teams"] == 2]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 2 teams): {df[df["Teams"] == 2]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (3 teams): {df[df["Teams"] == 3]["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT, 3 teams): {df[df["Teams"] == 3]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 3 teams): {df[df["Teams"] == 3]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (4 teams): {df[df["Teams"] == 4]["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT, 4 teams): {df[df["Teams"] == 4]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 4 teams): {df[df["Teams"] == 4]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (5 teams): {df[df["Teams"] == 5]["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT, 5 teams): {df[df["Teams"] == 5]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 5 teams): {df[df["Teams"] == 5]["Network Length (Star)"].mean()}')
    print(f'Mean Network Length (6 teams): {df[df["Teams"] == 6]["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT, 6 teams): {df[df["Teams"] == 6]["Network Length (ESMT)"].mean()}')
    print(f'Mean Network Length (Star, 6 teams): {df[df["Teams"] == 6]["Network Length (Star)"].mean()}')

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (2tRNP)', 'w = 12', 'w = 12 (2tRNP)', 'w = 18', 'w = 18 (2tRNP)', 'w = 24', 'w = 24 (2tRNP)']
    # legend_labels2 = ['t = 2', 't = 2 (2tRNP)', 't = 3', 't = 3 (2tRNP)', 't = 4', 't = 4 (2tRNP)', 't = 5', 't = 5 (2tRNP)', 't = 6', 't = 6 (2tRNP)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes2.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    # set font properties of colorbar label
    # cbar7 = axes7.collections[0].colorbar
    # cbar7.set_label("Ratio", fontdict={'size': 14})
    # cbar7.ax.tick_params(labelsize=14)
    # cbar8 = axes8.collections[0].colorbar
    # cbar8.set_label("Ratio", fontdict={'size': 14})
    # cbar8.ax.tick_params(labelsize=14)

    # cbar8 = axd9['r'].collections[0].colorbar
    # axd9['r'].set_ylabel("Ratio", fontdict={'size': 14})
    # axd9['r'].tick_params(labelsize=14)

    # for key, ax in axd9.items():
    #     if key == 'r':
    #         ax.spines['top'].set_visible(True)
    #         ax.spines['right'].set_visible(True)
    #     else:
    #         ax.spines['top'].set_visible(False)
    #         ax.spines['right'].set_visible(False)
    #     ax.spines['left'].set_visible(True)
    #     ax.spines['bottom'].set_visible(True)
    #     ax.spines['left'].set_linewidth(1)
    #     ax.spines['bottom'].set_linewidth(1)
    #     ax.tick_params(width=1)

    # find the maximum value in df_compare
    max_val = df_compare['Network Length'].max()
    # get the current maximum and minimum limits of the y-axis in axis4
    ymin, ymax = axes4.get_ylim()
    print('ymin:', ymin, 'ymax:', ymax)

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    # axes1.axis('equal')
    # axes2.axis('equal')

    # plt.show()
    # fig1.savefig(join(RESULTS_DIR, 'connectors_ex0.pdf'))
    # fig2.savefig(join(RESULTS_DIR, 'network_length_ex0.pdf'))  
    # fig3.savefig(join(RESULTS_DIR, 'network_length_ex0_star.pdf'))  
    fig4.savefig(join(RESULTS_DIR, 'network_length_comparison_star.pdf'))  

    # df_compare.to_csv(join(RESULTS_DIR, 'network_length.csv'), index=False)


def plot_network_length_real_robots():

    df_compare = pd.DataFrame({
        # 'Teams':                            pd.Series(dtype='int'), 
        # 'Workers Per Team':                  pd.Series(dtype='int'),
        'Solution':                            pd.Series(dtype='str'),
        'Network Length':                   pd.Series(dtype='float'),
        'ID':                               pd.Series(dtype='int'),
        # 'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        # 'Connectors':                       pd.Series(dtype='int'),
        # 'Connectors (ESMT)':   pd.Series(dtype='float'),
    })

    # Get trial names
    config_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    config_dirs.sort()

    print(config_dirs)

    for config in config_dirs:

        # if config[-1] != 'T':
        #     continue # don't include 'star'

        if config == 'our_approach':
            solution = 'Our approach'
            print('Our approach')
        elif config == 'star':
            solution = 'Our approach (without branch repositioning)'
            print('Starlike topology')

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, config)) if isdir(join(RESULTS_DIR, config, f))]
        trial_dirs.sort()

        print(trial_dirs)

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0

        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, config, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            # res_path_length = get_team_distance(graphs["res"])
            # esmt_path_length = get_team_distance(graphs["esmt"])
            # ttrnp_path_length = get_team_distance(graphs["2trnp"])

            # Add data of the successful trial
            # if not scenario in trial_no_points and not scenario in trial_no_connectivity:
            if not scenario in trial_no_connectivity:

                # if(s.numLeaders > 2):
                #     # # Sum
                #     # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Central Solver'], 'Network Length': [lengths['esmt']]})
                #     # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                #     # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Our Approach'], 'Network Length': [lengths['res']]})
                #     # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)
                #     # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': ['Star-like Topology'], 'Network Length': [lengths['star']]})
                #     # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                # Sum
                if solution == 'Our approach':
                    d = pd.DataFrame({'Solution': ['Steiner tree'], 'Network Length': [lengths['esmt']], 'ID': [count]})
                    df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                d = pd.DataFrame({'Solution': [solution], 'Network Length': [lengths['res']], 'ID': [count]})
                df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                if solution == 'Our approach':
                    d = pd.DataFrame({'Solution': ['Optimal starlike tree'], 'Network Length': [lengths['star']], 'ID': [count]})
                    df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

            else:
                print('skipping...')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            # num_to_use = 50
            # if (count % 10) % num_to_use == 0:
            #     count += 50 - num_to_use
            #     # print(count)
            #     # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig4 = plt.figure(figsize=(3.5, 3.8))
    axis4 = fig4.gca()

    # import matplotlib.pylab as pyplt

    # plt.rcParams['text.usetex'] = True

    # x = pyplt.linspace(0,5)
    # plt.plot(x,pyplt.sin(x))
    # plt.ylabel("This is "+r"$\sin(x)$", size=20)
    # plt.tight_layout()
    # plt.savefig('output.png')

    # exit(0)

    print(df_compare)

    # calculate the mean network length for each team and solution combination
    # df_compare = df4.groupby(['Teams', 'Network']).mean().reset_index()
    # print(df_compare)
    # print(df_compare.query('Solution=="Our Approach"')['Solution'])
    x_coords_steiner = np.array([df_compare.query('Solution=="Steiner tree"')['Solution'], df_compare.query('Solution=="Our approach"')['Solution']])
    y_coords_steiner = np.array([df_compare.query('Solution=="Steiner tree"')['Network Length'], df_compare.query('Solution=="Our approach"')['Network Length']])

    # x_coords = np.array([df_compare.query('Solution=="Our approach"')['Solution'], df_compare.query('Solution=="Starlike topology"')['Solution']])
    # y_coords = np.array([df_compare.query('Solution=="Our approach"')['Network Length'], df_compare.query('Solution=="Starlike topology"')['Network Length']])

    x_coords_star = np.array([df_compare.query('Solution=="Our approach"')['Solution'], df_compare.query('Solution=="Optimal starlike tree"')['Solution']])
    y_coords_star = np.array([df_compare.query('Solution=="Our approach"')['Network Length'], df_compare.query('Solution=="Optimal starlike tree"')['Network Length']])

    x_coords = np.array([df_compare.query('Solution=="Optimal starlike tree"')['Solution'], df_compare.query('Solution=="Our approach (without branch repositioning)"')['Solution']])
    y_coords = np.array([df_compare.query('Solution=="Optimal starlike tree"')['Network Length'], df_compare.query('Solution=="Our approach (without branch repositioning)"')['Network Length']])

    # y_coords = np.array([df_compare.query('Solution=="Our Approach"').Distance, df_compare.query('Solution=="Star-like topology"').Distance])
    
    # print(x_coords)
    # print(y_coords)

    # Loop x-coords annd y-coords to create a list of pairs
    # example: lines = [[(0, 11.44234246), (1, 12.05103481)]]
    lines_steiner = []
    for i in range(len(x_coords_steiner[0])):
        pair = [(0, y_coords_steiner[0][i]), (1, y_coords_steiner[1][i])]
        lines_steiner.append(pair)
    lines_star = []
    for i in range(len(x_coords_star[0])):
        pair = [(0, y_coords_star[0][i]), (1, y_coords_star[1][i])]
        lines_star.append(pair)
    lines = []
    for i in range(len(x_coords[0])):
        pair = [(0, y_coords[0][i]), (1, y_coords[1][i])]
        lines.append(pair)

    # g = sns.boxplot(data=df_compare, ax=axis4, x='Solution', y='Network Length', palette=['#66c2a5','#FC8D62','#e78ac3'])
    g = sns.boxplot(data=df_compare, ax=axis4, x='Solution', y='Network Length', palette='Set2')

    # get median values for each group
    medians = df_compare.groupby(['Solution'])['Network Length'].median()
    print(medians)

    # add scatter plot to same plot
    sns.stripplot(data=df_compare, ax=axis4, x='Solution', y='Network Length', color='black', size=5, jitter=False)

    # lc = LineCollection(lines)
    # lc.set_color((0.5, 0.5, 0.5))
    # axis4.add_collection(lc)

    # draw lines between plots
    for i in range(len(x_coords_steiner[0])):
        axis4.plot(x_coords_steiner[:, i], y_coords_steiner[:, i], color='#777777', linestyle='--', lw=1)
    for i in range(len(x_coords_star[0])):
        axis4.plot(x_coords_star[:, i], y_coords_star[:, i], color='#777777', linestyle='--', lw=1)
    for i in range(len(x_coords[0])):
        axis4.plot(x_coords[:, i], y_coords[:, i], color='#777777', linestyle='--', lw=1)

    # add stats annotation between the two groups
    # # stats annotation: resuires seaborn <= 0.12.2 !!
    # pairs = [('Our approach', 'Starlike topology')]
    # annotator = Annotator(axis4, pairs, data=df_compare, x='Solution', y='Network Length', order=['Our approach', 'Starlike topology'])
    # annotator.configure(test='Wilcoxon', text_format='star', loc='inside')
    # annotator.apply_and_annotate()
        
    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(10)

    # Set axis labels
    # matplotlib.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
    # matplotlib.rc('text', usetex=True)
    # plt.rcParams['text.usetex'] = True
    # plt.rcParams["mathtext.default"]
    # axis4.set_ylabel('Network length / '+r'$n$'+' (m)')
    axis4.set_ylabel('Network length (m)', fontproperties=font)
    # axis4.set_xlabel('Solution', fontproperties=font)

    # remove x label
    axis4.set_xlabel('')

    # move axis4 xlabel slightly down
    # axis4.xaxis.labelpad = 10

    # Get the current x-axis tick labels
    current_labels = plt.xticks()[1]

    # Modify the labels to include newline character
    new_labels = [label.get_text().replace(' ', '\n') for label in current_labels]

    # Set the modified labels as new x-axis tick labels
    plt.xticks(range(len(new_labels)), new_labels)

    # for ax in axes:
    axis4.spines['top'].set_visible(False)
    axis4.spines['right'].set_visible(False)
    axis4.spines['left'].set_visible(True)
    axis4.spines['bottom'].set_visible(True)
    axis4.spines['left'].set_linewidth(1)
    axis4.spines['bottom'].set_linewidth(1)
    axis4.tick_params(width=1)

    for label in axis4.get_xticklabels():
        label.set_fontproperties(font2)
    for label in axis4.get_yticklabels():
        label.set_fontproperties(font2)

    # border and ticks
    # max_val = max(df['Connectors'].max(),df['Connectors (ESMT)'].max())
    # axes1.set_xticks(np.arange(0, max_val+2, 1))
    # axes1.set_yticks(np.arange(0, max_val+2, 1))
    # axes1.set_xlim([0,max_val+2])
    # axes1.set_ylim([0,max_val+2])

    # max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    # axes2.set_xticks(np.arange(0, max_val+2, 1))
    # axes2.set_yticks(np.arange(0, max_val+2, 1))
    # axes2.set_xlim([0,max_val+2])
    # axes2.set_ylim([0,max_val+2])

    # # max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    # axes3.set_xticks(np.arange(0, max_val+2, 1))
    # axes3.set_yticks(np.arange(0, max_val+2, 1))
    # axes3.set_xlim([0,max_val+2])
    # axes3.set_ylim([0,max_val+2])

    # print(df.to_string())
    # print(f'Mean Connectors: {df["Connectors"].mean()}')
    # print(f'Mean Connectors (ESMT): {df["Connectors (ESMT)"].mean()}')
    # print(f'Mean Network Length: {df["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT): {df["Network Length (ESMT)"].mean()}')
    # print(f'Mean Network Length (Star): {df["Network Length (Star)"].mean()}')

    # # print mean network length
    # print(f'Mean Network Length (4 teams): {df[df["Teams"] == 4]["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT, 4 teams): {df[df["Teams"] == 4]["Network Length (ESMT)"].mean()}')
    # print(f'Mean Network Length (Star, 4 teams): {df[df["Teams"] == 4]["Network Length (Star)"].mean()}')

    # Print average network length for each solution
    # Calculate mean network length for 'Our approach'
    mean_our_approach = df_compare[df_compare['Solution'] == 'Our approach']['Network Length'].mean()

    # Calculate mean network length for 'Starlike topology'
    mean_starlike_topology = df_compare[df_compare['Solution'] == 'Starlike topology']['Network Length'].mean()

    # Print the mean network lengths
    print(f'Mean Network Length (Our approach): {mean_our_approach}')
    print(f'Mean Network Length (Starlike topology): {mean_starlike_topology}')

    fig4.tight_layout()

    fig4.savefig(join(RESULTS_DIR, 'network_length_comparison_real_robot.pdf'))  

    df_compare.to_csv(join(RESULTS_DIR, 'network_length_real_robot.csv'), index=False)


def plot_num_connectors():

    df_compare = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        'Solution':                            pd.Series(dtype='str'),
        'Number of Connectors':                   pd.Series(dtype='float'),
        'Number of Workers':                   pd.Series(dtype='float'),
    })

    results_dir = RESULTS_DIR
    # results_dir = join(RESULTS_DIR, '0.80RAB')

    # Get trial names
    team_num_dirs = [f for f in listdir(results_dir) if isdir(join(results_dir, f))]
    team_num_dirs.sort()

    print(team_num_dirs)

    for team_num in team_num_dirs:

        # solution
        if 'star' in team_num:
            solution = 'Our approach (without branch repositioning)'
        else:
            solution = 'Our approach'

        trial_dirs = [f for f in listdir(join(results_dir, team_num)) if isdir(join(results_dir, team_num, f))]
        trial_dirs.sort()

        # print(trial_dirs)

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0

        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(results_dir, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distance(graphs["res"])
            esmt_path_length = get_team_distance(graphs["esmt"])
            # ttrnp_path_length = get_team_distance(graphs["2trnp"])

            # Add data of the successful trial
            # if not scenario in trial_no_points and not scenario in trial_no_connectivity:
            if not scenario in trial_no_connectivity:

                if s.numLeaders > 2:
                    # Sum
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': [solution], 'Number of Connectors': [connectors['res']], 'Number of Workers': [(s.numWorkers-connectors['res'])]})
                    # df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                    # Average
                    d = pd.DataFrame({'Teams': [s.numLeaders], 'Solution': [solution], 'Number of Connectors': [connectors['res']/s.numLeaders], 'Number of Workers': [(s.numWorkers-connectors['res'])/s.numLeaders]})
                    df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

            else:
                print('skipping...')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            # num_to_use = 50
            # if (count % 10) % num_to_use == 0:
            #     count += 50 - num_to_use
            #     # print(count)
            #     # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig4 = plt.figure(figsize=(5, 6))
    axes4 = fig4.gca()
    fig5 = plt.figure(figsize=(5, 6))
    axes5 = fig5.gca()

    axes = [axes4, axes5]

    # order_num_team = list(df['Teams'].unique())
    # order_num_team.sort()
    # order_team_size = list(df['Workers Per Team'].unique())
    # order_team_size.sort()    
    
    # g1 = sns.boxplot(data=df_compare, ax=axes4, x='Teams', y='Number of Connectors', hue='Solution', palette=['#FC8D62','#8da0cb'])
    g1 = sns.boxplot(data=df_compare, ax=axes4, x='Teams', y='Number of Connectors', hue='Solution', palette=['#FC8D62','#e78ac3' ])
    print(df_compare)
    # g2 = sns.boxplot(data=df_compare, ax=axes5, x='Teams', y='Number of Workers', hue='Solution', palette=['#FC8D62','#8da0cb'])
    g2 = sns.boxplot(data=df_compare, ax=axes5, x='Teams', y='Number of Workers', hue='Solution', palette=['#FC8D62','#e78ac3' ])

    g = [g1,g2]

    # https://stackoverflow.com/questions/72656861/how-to-add-hatches-to-boxplots-with-sns-boxplot-or-sns-catplot
    # hatches must equal the number of hues (3 in this case)
    hatches = ['..', 'x']

    # iterate through each subplot / Facet
    for gx in g:
        for ax in [gx.axes]:

            # select the correct patches
            patches = [patch for patch in ax.patches if type(patch) == PathPatch]
            # the number of patches should be evenly divisible by the number of hatches
            print('patches:', len(patches))
            print('hatches:', len(hatches))
            # create a list ['//', '//', '//', '//', '..', '..', '..', '..', 'xx', 'xx', 'xx', 'xx'] using a loop
            # multiply the hatches by the number of boxes in the plot

            h = int(len(patches) / len(hatches))
            print('h:',h)
            result = [hatch for hatch in hatches for _ in range(h)]
            print(result)
            print('h:', h)
            # iterate through the patches for each subplot
            for patch, hatch in zip(patches, result):
                patch.set_hatch(hatch)
                # fc = patch.get_facecolor()
                patch.set_edgecolor('black')

                # change median line to also be black
                for line in ax.lines:
                    line.set_color('black')

    # place the legend on the top of the plot and place each item horizontally
    legend1 = axes4.legend(loc='upper center', bbox_to_anchor=(0.44, 1.18), ncol=1)
    legend2 = axes5.legend(loc='upper center', bbox_to_anchor=(0.44, 1.18), ncol=1)
    
    legend = [legend1, legend2]

    # Make the legend patches also match the box plot appearances
    for l in legend:
        for lp, hatch in zip(l.get_patches(), hatches):
            lp.set_hatch(hatch)
            lp.set_edgecolor('black')

        # change the font of the text in the legend
        for text in l.get_texts():
            text.set_fontsize(16)

    # run mann whitney test to check whether the results of Network and Network (star) are statistically significant

    # Change the color of the mean line to Crimson

    # for line in axes4.get_lines():
    #     # check if the line is a dash
    #     if line.get_linestyle() == '--':
    #         line.set_color('Crimson')
    
    # axes4.plot([], [], '--', linewidth=1, color='Crimson', label='mean')
    # axes4.legend()

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(18)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(16)

    # Set axis labels
    plt.rcParams['text.usetex'] = True
    axes4.set_ylabel(r'Number of connectors / $n$', fontproperties=font)
    axes4.set_xlabel(r'Number of locations $n$', fontproperties=font)

    axes5.set_ylabel(r'Number of workers / $n$', fontproperties=font)
    axes5.set_xlabel(r'Number of locations $n$', fontproperties=font)

    # max_connectors = df_compare['Number of Connectors'].max()
    # axes4.set_yticks(np.arange(0, max_connectors+2, 4))

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)


    # print(df.to_string())
    # print(f'Mean Connectors: {df["Connectors"].mean()}')
    # print(f'Mean Connectors (ESMT): {df["Connectors (ESMT)"].mean()}')
    # print(f'Mean Network Length: {df["Network Length"].mean()}')
    # print(f'Mean Network Length (ESMT): {df["Network Length (ESMT)"].mean()}')
    # print(f'Mean Network Length (Star): {df["Network Length (Star)"].mean()}')

    # print number of connectors
    # print the number of connectors for each team and solution combination
    

    # print(f'Mean Number of Connectors (3 teams): {df_compare[df_compare["Teams"] == 3 and df_compare["Solution"] == "Our Approach"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (Star, 3 teams): {df_compare[df_compare["Teams"] == 3 and df_compare["Solution"] == "Star-like Topology"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (4 teams): {df_compare[df_compare["Teams"] == 4 and df_compare["Solution"] == "Our Approach"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (Star, 4 teams): {df_compare[df_compare["Teams"] == 4 and df_compare["Solution"] == "Star-like Topology"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (5 teams): {df_compare[df_compare["Teams"] == 5 and df_compare["Solution"] == "Our Approach"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (Star, 5 teams): {df_compare[df_compare["Teams"] == 5 and df_compare["Solution"] == "Star-like Topology"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (6 teams): {df_compare[df_compare["Teams"] == 6 and df_compare["Solution"] == "Our Approach"]["Number of Connectors"].mean()}')
    # print(f'Mean Number of Connectors (Star, 6 teams): {df_compare[df_compare["Teams"] == 6 and df_compare["Solution"] == "Star-like Topology"]["Number of Connectors"].mean()}')
    fig4.tight_layout()
    # # cut the top and bottom of the figure slightly
    # plt.subplots_adjust(top=0.9, bottom=0.1)
    fig5.tight_layout()
    # axes1.axis('equal')
    # axes2.axis('equal')

    # plt.show()
    fig4.savefig(join(results_dir, 'num_connectors_comparison.pdf'))  
    fig5.savefig(join(results_dir, 'num_workers_comparison.pdf'))


def plot_num_connectors_comm():
    
    df_compare = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        'Communication Range':                  pd.Series(dtype='float'),
        'Solution':                            pd.Series(dtype='str'),
        'Number of Connectors':                   pd.Series(dtype='float'),
        'Number of Workers':                   pd.Series(dtype='float'),
    })

    comm_range_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    comm_range_dirs.sort()

    print(comm_range_dirs)

    for comm_range in comm_range_dirs:

        global COMM_RANGE
        # remove characters from comm_range and make it a double
        if 'RAB' in comm_range:
            COMM_RANGE = float(comm_range.replace('RAB', ''))
        else:
            continue

        # print(COMM_RANGE)

        # Get the communication range value
        if 'RAB' in comm_range:
            comm_range_val = float(comm_range.replace('RAB', ''))

        # Get trial names
        team_num_dirs = [f for f in listdir(join(RESULTS_DIR, comm_range)) if isdir(join(RESULTS_DIR, comm_range, f))]
        team_num_dirs.sort()

        print(team_num_dirs)

        for team_num in team_num_dirs:

            # solution
            solution = 'Our Approach'

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, comm_range, team_num)) if isdir(join(RESULTS_DIR, comm_range, team_num, f))]
            trial_dirs.sort()

            # print(trial_dirs)

            trial_no_points = {}
            trial_no_connectivity = {}
            trial_no_team_connectivity = {}

            start_time = time.time()

            count = 0

            # for scenario in trial_dirs:
            while count < len(trial_dirs):
                # if count >= len(trial_dirs):
                #     break

                scenario = trial_dirs[count]

                start_time_single = time.time()

                # Check if the trial was successful
                s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, comm_range, team_num, scenario))
                if final_points == 0:
                    trial_no_points[scenario] = {
                        'seed': s.seed,
                        'data': final_points
                    }

                if final_team_connectivity:
                    trial_no_team_connectivity[scenario] = {
                        'seed': s.seed,
                        'data': final_team_connectivity
                    }

                # if final_points > 0 and final_connectivity:

                # Find the total network length and connectors for the result obtained and the solver output
                graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
                if not nx.is_connected(graphs['res']):
                    trial_no_connectivity[scenario] = {
                        'seed': s.seed,
                        'data': False
                    }

                # Find the total distance between every team along the network
                # get_team_distance(graphs['res'])

                res_path_length = get_team_distance(graphs["res"])
                esmt_path_length = get_team_distance(graphs["esmt"])
                # ttrnp_path_length = get_team_distance(graphs["2trnp"])

                # Add data of the successful trial
                # if not scenario in trial_no_points and not scenario in trial_no_connectivity:
                if not scenario in trial_no_connectivity:

                    # Sum
                    d = pd.DataFrame({'Teams': [s.numLeaders], 'Communication Range': [comm_range_val], 'Solution': [solution], 'Number of Connectors': [connectors['res']], 'Number of Workers': [(s.numWorkers-connectors['res'])]})

                    # Average
                    # d = pd.DataFrame({'Teams': [s.numLeaders], 'Communication Range': [comm_range_val], 'Solution': [solution], 'Number of Connectors': [connectors['res']], 'Number of Workers': [(s.numWorkers-connectors['res'])]})
                    df_compare = pd.concat([df_compare, d], ignore_index=True, axis=0)

                else:
                    print('skipping...')

                duration_single = round(time.time() - start_time_single, 3)
                duration_total = round(time.time() - start_time, 3)
                # print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

                # DEBUG: For limiting the number of data to plot
                count += 1
                # num_to_use = 50
                # if (count % 10) % num_to_use == 0:
                #     count += 50 - num_to_use
                #     # print(count)
                #     # break
                # if count >= 1:
                #     break

            duration = round(time.time() - start_time, 3)
            print('Finished in {0} seconds'.format(duration))

            # print(f'### No points scored ###')
            # for scenario, trial in trial_no_points.items():
            #     print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
            # print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

            print(f'### Lost global connectivity ###')
            for scenario, trial in trial_no_connectivity.items():
                print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
            print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

            # print(f'### Lost team connectivity ###')
            # for scenario, trial in trial_no_team_connectivity.items():
            #     print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
            # print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig4 = plt.figure(figsize=(6, 3))
    axes4 = fig4.gca()
    fig5 = plt.figure(figsize=(6, 3))
    axes5 = fig5.gca()

    axes = [axes4, axes5]

    # order_num_team = list(df['Teams'].unique())
    # order_num_team.sort()
    # order_team_size = list(df['Workers Per Team'].unique())
    # order_team_size.sort()    
    
    g1 = sns.boxplot(data=df_compare, ax=axes4, x='Communication Range', y='Number of Connectors', hue='Teams', palette='Set1')
    print(df_compare)
    g2 = sns.boxplot(data=df_compare, ax=axes5, x='Communication Range', y='Number of Workers', hue='Teams', palette='Set1')

    g = [g1,g2]

    # https://stackoverflow.com/questions/72656861/how-to-add-hatches-to-boxplots-with-sns-boxplot-or-sns-catplot
    # hatches must equal the number of hues (3 in this case)
    hatches = ['//', '..', 'xx', '']
    # hatches = ['//', '..', 'xx', '++', 'oo', '||']
    # all types of hatches
    # hatches = ['/', '\\', '|', '-', '+', 'x', 'o', 'O', '.', '*']

    # iterate through each subplot / Facet
    for gx in g:
        for ax in [gx.axes]:

            # select the correct patches
            patches = [patch for patch in ax.patches if type(patch) == PathPatch]
            # the number of patches should be evenly divisible by the number of hatches
            print('patches:', len(patches))
            print('hatches:', len(hatches))
            # create a list ['//', '//', '//', '//', '..', '..', '..', '..', 'xx', 'xx', 'xx', 'xx'] using a loop
            # multiply the hatches by the number of boxes in the plot

            h = int(len(patches) / len(hatches))
            print('h:',h)
            result = [hatch for hatch in hatches for _ in range(h)]
            print(result)
            print('h:', h)
            # iterate through the patches for each subplot
            for patch, hatch in zip(patches, result):
                patch.set_hatch(hatch)
                # fc = patch.get_facecolor()
                patch.set_edgecolor('black')

                # change median line to also be black
                for line in ax.lines:
                    line.set_color('black')

    # place the legend on the top of the plot and place each item horizontally
    legend1 = axes4.legend(loc='upper right', ncol=4, bbox_to_anchor=(1.0, 1.1))
    # legend1 = axes4.legend()
    # legend2 = axes5.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3)
    legend2 = axes5.legend()

    legend = [legend1, legend2]

    # Make the legend patches also match the box plot appearances
    for l in legend:
        for lp, hatch in zip(l.get_patches(), hatches):
            # lp.set_label('Teams')
            lp.set_hatch(hatch)
            lp.set_edgecolor('black')

    # for line in axes4.get_lines():
    #     # check if the line is a dash
    #     if line.get_linestyle() == '--':
    #         line.set_color('Crimson')
    
    # axes4.plot([], [], '--', linewidth=1, color='Crimson', label='mean')
    # axes4.legend()

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(14)

    # Set axis labels
    axes4.set_ylabel('Number of connectors', fontproperties=font)
    axes4.set_xlabel('Communication range ' + r'$r_{\mathrm{com}}$' + ' (m)', fontproperties=font)

    axes5.set_ylabel('Number of workers', fontproperties=font)
    axes5.set_xlabel('Communication range (m)', fontproperties=font)

    # Get max number of connectors in df_compare
    max_connectors = df_compare['Number of Connectors'].max()
    max_workers = df_compare['Number of Workers'].max()

    axes4.set_yticks(np.arange(0, max_connectors+1, 5))
    axes5.set_yticks(np.arange(0, max_workers+1, 5))

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

        # set legend font
        for l in legend:
            for label in l.get_texts():
                label.set_fontproperties(font2)
            # for label in l.get_lines():
            #     label.set_linewidth(1.5)
                
            l.set_title(r'Number of locations $n$', prop=font2)
    
    # Set y_lim between 0 and max value of number of connectors
    axes4.set_ylim(0, max_connectors+1)
    axes5.set_ylim(0, max_workers+1)

    fig4.tight_layout()
    fig5.tight_layout()

    # fig4.subplots_adjust(hspace=0.05)
    # fig4.subplots_adjust(top=0.9, bottom=0.1)

    # plt.show()
    fig4.savefig(join(RESULTS_DIR, 'num_connectors_comm.pdf'))  
    fig5.savefig(join(RESULTS_DIR, 'num_workers_comm.pdf'))


def find_team_distances_average(plot=False):

    df = pd.DataFrame({
        'min':          pd.Series(dtype='float'),  
        'max':          pd.Series(dtype='float'),
        'mean':         pd.Series(dtype='float'),
        'diff':         pd.Series(dtype='float'),
        'min esmt':    pd.Series(dtype='float'),  
        'max esmt':    pd.Series(dtype='float'),
        'mean esmt':   pd.Series(dtype='float'),
        'diff esmt':   pd.Series(dtype='float'),
    })
    
    # Get trial names
    variation_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    variation_dirs.sort()

    for variation in variation_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]
            delay = int(scenario.split('_')[3][:-1]) # Get send delay

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, variation, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=plot)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            res_dist = get_team_distances(graphs['res'])
            esmt_dist = get_team_distances(graphs['esmt'])

            # Add data of the successful trial
            if not scenario in trial_no_connectivity and s.totalTime != 6000:
                
                d = pd.DataFrame({
                    'min': [res_dist['min']], 
                    'max': [res_dist['max']], 
                    'mean': [res_dist['mean']],
                    'diff': [res_dist['diff']],
                    'min esmt': [esmt_dist['min']], 
                    'max esmt': [esmt_dist['max']],
                    'mean esmt': [esmt_dist['mean']], 
                    'diff esmt': [esmt_dist['diff']],
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

            else:
                print(f'fail? {scenario}')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # print(df)

            # DEBUG: For limiting the number of data to plot
            count += 1
            num_to_use = 50
            if (count % 10) % num_to_use == 0:
                count += 50 - num_to_use
                # print(count)
                # break

    df_melted = pd.melt(df, value_vars=['min', 'max', 'min esmt', 'max esmt'], var_name='column')

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font2.set_size(12)

    fig1 = plt.figure(figsize=(7,5))
    axis1 = fig1.gca()

    # Set1
    color1 = (0.8941176470588236, 0.10196078431372549, 0.10980392156862745)
    color2 = (0.21568627450980393, 0.49411764705882355, 0.7215686274509804)
    # Set2
    # color1 = (0.4, 0.7607843137254902, 0.6470588235294118)
    # color2 = (0.9882352941176471, 0.5529411764705883, 0.3843137254901961)
    strip1 = sns.stripplot(data=df_melted[df_melted['column'].str.contains('min')], ax=axis1, x='column', y='value', color=color2)
    strip2 = sns.stripplot(data=df_melted[df_melted['column'].str.contains('max')], ax=axis1, x='column', y='value', color=color1, marker=(4,0,0))

    filtered_df = df.filter(['mean', 'mean esmt'])
    df_melted = filtered_df.melt(var_name='Columns', value_name='Values')

    sns.boxplot(data=df_melted, x='Columns', y='Values', ax=axis1,
                showmeans=True,
                meanline=True,
                meanprops={'color': 'k', 'ls': '-', 'lw': 1},
                medianprops={'visible': False},
                whiskerprops={'visible': False},
                zorder=10,
                showfliers=False,
                showbox=False,
                showcaps=False)

    print(df)
    print('max,max_esmt')
    for i, n in enumerate(df['max']):
        print(f'{df["max"][i]},{df["max esmt"][i]}')
    # print('mean esmt')
    # for i in df['mean esmt']:
    #     print(i)
    # print()
    print(df['mean'].mean())
    print(df['mean esmt'].mean())

    strip1.set_xticklabels([])
    strip2.set_xticklabels([])
    plt.xticks([0, 1], ['trial results', 'ESMT'])

    axis1.set_xlabel('', fontproperties=font)
    axis1.set_ylabel('Path length (m)', fontproperties=font)

    axis1.spines['top'].set_visible(False)
    axis1.spines['right'].set_visible(False)
    axis1.spines['left'].set_visible(True)
    axis1.spines['bottom'].set_visible(True)
    axis1.spines['left'].set_linewidth(1)
    axis1.spines['bottom'].set_linewidth(1)
    axis1.tick_params(width=1)

    for label in axis1.get_xticklabels():
        label.set_fontproperties(font)
    for label in axis1.get_yticklabels():
        label.set_fontproperties(font2)

    # border and ticks
    # axes1.set_yticks(np.arange(2, 6+1, 1))
    axis1.set_ylim([0,df['max esmt'].max()+0.1])

    fig1.tight_layout()

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'path_length.pdf'))


def main():

    ### Scalability analysis
    # plot_all_trials()
    
    ### Congestion analysis
    # plot_all_trials_congestion('cross', 4100) # {cross|line}
    # plot_all_trials_congestion('line', 4100) # {cross|line}

    # plot_network_length()
    # plot_network_length_star()
    # plot_network_length_real_robots()
    # plot_num_connectors()
    # plot_num_connectors_comm()

    # find_team_distances_average(plot=False)

    # Load a single trial
    # scenario = 'nop_ex0_4T_6R_046'
    # scenario = 'nop_ex0_4T_6R_039'

    # scenario = 'nop_ex1_4T_6R_008'

    ### final timestep
    # scenario = 'network_maintenance_3T_6R_019'
    scenario = 'network_maintenance_6T_12R_0.8RAB_001'
    path = join(environ['HOME'], f'<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance', scenario)
    # path = join(environ['HOME'], f'<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_iros/{scenario[20:22]}', scenario)
    real_robot = False

    ### comm_range
    # comm_range = comm_range_name = '0.55'
    # if comm_range[-1] == '0':
    #     comm_range_name = comm_range[:-1]
    # scenario = f'network_maintenance_6T_12R_{comm_range_name}RAB_092'
    # path = join(environ['HOME'], f'<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_comm/{comm_range}RAB/{scenario[20:22]}', scenario)
    # real_robot = False
    # global RESULTS_DIR
    # RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_comm/')

    # load_log_with_checks(path, print_result=True)

    ### real robot
    # scenario = 'results_008'
    # solution = 'star'
    # path = join(environ['HOME'], f'<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_real_robots/', solution, scenario)
    # real_robot = True
    # SUMMARY_FILENAME = ''
    # RESULTS_DIR = join(environ['HOME'], '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_real_robots/')

    ### plot position at final timestep
    plot_single_trial_at_time(path, plot=True, final_timestep=True, real_robot=real_robot)


if __name__ == "__main__":
    main()
