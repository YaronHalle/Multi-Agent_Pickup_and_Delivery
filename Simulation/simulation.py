import random
import argparse
import yaml
import json
import os
from Simulation.TP_with_recovery import TokenPassingRecovery
import RoothPath


class Simulation(object):
    def __init__(self, tasks, agents, solver):
        self.tasks = tasks
        self.agents = agents
        self.solver = solver
        self.time = 0
        self.start_times = []
        self.actual_paths = {}  # Tracks the actual paths agent took, regardless of solver decisions
        self.statistics = {}
        self.N_completed_tasks = 0
        self.initialize_simulation()

    def simulation_ended(self):
        return self.N_completed_tasks == len(self.tasks)

    def initialize_simulation(self):
        for t in self.tasks:
            self.start_times.append(t['start_time'])
        for agent in self.agents:
            self.actual_paths[agent['name']] = [{'t': 0, 'x': agent['start'][0], 'y': agent['start'][1]}]

    def time_forward(self, algorithm):
        self.time = self.time + 1
        print('Time:', self.time)
        algorithm.time_forward()
        agents_to_move = self.agents
        random.shuffle(agents_to_move)
        for agent in agents_to_move:
            current_agent_pos = self.actual_paths[agent['name']][-1]
            if self.delays is not None:
                if self.time in self.delays[agent['name']]:
                    self.actual_paths[agent['name']].append(
                        {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})
                else:
                    self.actual_paths[agent['name']].append(
                        {'t': self.time, 'x': algorithm.get_token()['agents'][agent['name']][0][0],
                         'y': algorithm.get_token()['agents'][agent['name']][0][1]})
            elif self.delays_now > 0:
                self.delays_now = self.delays_now - 1
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})
            else:
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': algorithm.get_token()['agents'][agent['name']][0][0],
                     'y': algorithm.get_token()['agents'][agent['name']][0][1]})

    def get_time(self):
        return self.time

    def get_actual_paths(self):
        return self.actual_paths

    def get_new_tasks(self):
        new = []
        for t in self.tasks:
            if t['start_time'] == self.time:
                new.append(t)
        return new

    def move_agents(self):
        return 0

    def compute_statistics(self):
        return 0

#
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument('-param', help='Input file containing map and obstacles')
#     parser.add_argument('-output', help='Output file with the schedule')
#     args = parser.parse_args()
#
#     if args.param is None:
#         with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
#             config = json.load(json_file)
#         args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))
#         args.output = os.path.join(RoothPath.get_root(), 'output.yaml')
#
#     # Read from input file
#     with open(args.param, 'r') as param_file:
#         try:
#             param = yaml.load(param_file, Loader=yaml.FullLoader)
#         except yaml.YAMLError as exc:
#             print(exc)
#
#     dimensions = param['map']['dimensions']
#     obstacles = param['map']['obstacles']
#     non_task_endpoints = param['map']['non_task_endpoints']
#     agents = param['agents']
#     tasks = param['tasks']
#     delays = param['delays']
#
#     # Simulate
#     simulation = Simulation(tasks, agents, delays=delays)
#     tp = TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, simulation, a_star_max_iter=2000, k=5)
#     while tp.get_completed_tasks() != len(tasks):
#         simulation.time_forward(tp)
#
#     cost = 0
#     for path in simulation.actual_paths.values():
#         cost = cost + len(path)
#     output = {'schedule': simulation.actual_paths, 'cost': cost, 'completed_tasks_times': tp.get_completed_tasks_times(),
#               'n_replans': tp.get_n_replans()}
#     with open(args.output, 'w') as output_yaml:
#         yaml.safe_dump(output, output_yaml)
