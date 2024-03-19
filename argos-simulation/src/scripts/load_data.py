import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "protos", "generated")) # Path to compiled proto files
import time_step_pb2
from google.protobuf.internal.decoder import _DecodeVarint32

import pandas as pd

class SimData:
    """Wrapper class for parsing protobuf experiment data."""

    def __init__(self, log_data_path, summary_path='', commands_path=''):
        """Constructor"""

        # Data dict
        self.data = {}

        self.read_log_data(log_data_path)

        # Add experiment summary data if csv file exists
        if summary_path:
            self.read_summary(summary_path)
        else:
            self.seed = None
            self.scenarioName = ''
            self.maxSimulationClock = None
            self.arenaRadius = 0
            self.deployRadius = 0

        # Add user comamnds if csv file exists
        if commands_path:
            self.read_commands(commands_path)


    def read_log_data(self, log_data_path):
        """Parses a protobuf file with simulation logs."""

        with open(log_data_path, 'rb') as f:
            buf = f.read()
            n = 0

            # Read each timestep
            while n < len(buf):
                msg_len, new_pos = _DecodeVarint32(buf, n)
                n = new_pos
                msg_buf = buf[n:n+msg_len]
                n += msg_len
                read_data = time_step_pb2.TimeStep()
                read_data.ParseFromString(msg_buf)

                # Create an entry for the timestep
                self.data[read_data.time] = {}
                self.data[read_data.time]['log'] = read_data
                self.data[read_data.time]['commands'] = [] # Create empty list of commands
                
            # Store experiment summary
            if self.data:

                # Total timesteps
                self.totalTime = len(self.data)

                # Points scored
                if not self.totalTime in self.data:
                    self.totalTime -= 1 # Real robot experiments
                self.totalPoints = self.data[self.totalTime]['log'].points

                # Number of leaders and robots
                leaderNames = []
                teamNames = []
                for robot in self.data[1]['log'].robots:
                    if robot.state == time_step_pb2.Robot.LEADER:
                        leaderNames.append(robot.name)
                        teamNames.append(robot.teamID)

                self.leaders = leaderNames
                self.teams = teamNames
                self.numLeaders = len(self.leaders)
                self.numWorkers = len(self.data[1]['log'].robots) - self.numLeaders

                # Number of tasks that appeared in the experiment
                maxId = 0
                for task in self.data[self.totalTime]['log'].tasks:
                    id = int(task.name[5:])
                    if id > maxId:
                        maxId = id

                self.numTasks = maxId


    def read_summary(self, summary_path):
        """Parses a csv file with experiment information."""

        df = pd.read_csv(summary_path, index_col=0, names=['colA'], header=None)
        df = df.T

        self.scenarioName = df['SCENARIO_NAME'].iloc[0]
        self.seed = df['SEED'].iloc[0]
        self.maxSimulationClock = float(df['MAX_SIMULATION_CLOCK'].iloc[0])
        if 'ARENA_RADIUS' in df:
            self.arenaRadius = float(df['ARENA_RADIUS'].iloc[0])
        if 'DEPLOY_RADIUS' in df:
            self.deployRadius = float(df['DEPLOY_RADIUS'].iloc[0])
        if 'ROBOTS COLLIDED' in df:
            self.robotsCollided = int(df['ROBOTS COLLIDED'].iloc[0])
        # except (ValueError, KeyError, IndexError) as e:
        #     print("'ROBOTS COLLIDED' does not exist. Skipping..")


    def read_commands(self, commands_path):
        """Parses a csv file with user commands."""

        df = pd.read_csv(commands_path)
        # print(df.to_string())

        self.users = set()

        # iterate row
            # append command (type, value, user, robot) to time

        for ind in df.index:
            command = {}

            time = df['TIME'][ind]
            command['time'] = time
            command['type'] = df['COMMAND'][ind]
            command['user'] = df['USER'][ind]
            command['value'] = df['VALUE'][ind]

            # Assume valid username is less than 8 characters
            if len(str(command['user'])) < 8:
                self.data[time]['commands'].append(command)
                self.users.add(int(command['user']))

            # print(self.data[time]['commands'])


    def __getitem__(self, key):
        return self.data[key]
    

if __name__ == "__main__":
    log_data_path = '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/network_maintenance_comm/0.60RAB/5T/network_maintenance_5T_12R_0.6RAB_029/log_data.pb'
    commands_path = '<path-to-file>/minimal-length-swarm-networks/argos-simulation/results/iros/5T/nop_ex1_5T_6R_009/summary.csv'
    # print if commands_path exists
    if not os.path.exists(commands_path):
        print('No commands file found')
    
    s = SimData(log_data_path)

    print('Total time: {}'.format(s.totalTime))
    print('Points: {}'.format(s.data[s.totalTime]['log'].points))
    print('Number of leaders: {}'.format(s.numLeaders))
    # for leader in s.leaders:
    #     print('\t{}'.format(leader))
    print('Teams:')
    for team in s.teams:
        print('\t{}'.format(team))
    print('Number of followers: {}'.format(s.numWorkers))
    print('Number of tasks: {}'.format(s.numTasks))

    print('arenaRadius:',s.arenaRadius)
