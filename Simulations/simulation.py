import random
import argparse
import yaml
import json
import os
#from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulations.central_algorithm import Task, TaskState
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
        self.simulation_end_time = None
        self.free_non_task_endpoints = []
        self.occupied_non_task_endpoints = []

    def simulation_ended(self):
        #return (self.N_completed_tasks == len(self.tasks)) or (self.time >= self.simulation_end_time)
        return self.time >= self.simulation_end_time

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

    def generate_new_tasks(self, prev_tasks, starts, goals, n_tasks):
        new_tasks = []
        maximal_attemps = 100
        n_tasks_so_far = len(prev_tasks)
        taken_squares = set()
        for task in prev_tasks.values():
            if task.task_state == TaskState.PENDING or task.task_state == TaskState.ASSIGNED:
                taken_squares.add(tuple([task.start_pos[0], task.start_pos[1]]))
                taken_squares.add(tuple([task.goal_pos[0], task.goal_pos[1]]))
            if task.task_state == TaskState.EXECUTED:
                taken_squares.add(tuple([task.goal_pos[0], task.goal_pos[1]]))

        for i in range(n_tasks):
            next_start = random.choice(starts)
            next_goal = random.choice(goals)

            current_attempt = 0
            while (tuple([next_start[0], next_start[1]]) in taken_squares or
                    tuple([next_goal[0], next_goal[1]]) in taken_squares) and \
                    current_attempt < maximal_attemps:
                next_start = random.choice(starts)
                next_goal = random.choice(goals)
                current_attempt += 1

            taken_squares.add(tuple([next_start[0], next_start[1]]))
            taken_squares.add(tuple([next_goal[0], next_goal[1]]))

            new_task = Task()
            new_task.task_name = 'task' + str(n_tasks_so_far + i)
            new_task.start_pos = next_start
            new_task.goal_pos = next_goal
            new_task.task_state = TaskState.PENDING
            new_task.task_type = 0
            new_task.start_time = int(self.time)
            new_tasks.append(new_task)

        return new_tasks

    def get_new_tasks(self):
        new = []
        for t in self.tasks:
            if t['start_time'] == self.time:
                new_task = Task()
                new_task.task_name = t['task_name']
                new_task.start_pos = t['start']
                new_task.goal_pos = t['goal']
                new_task.task_state = TaskState.PENDING
                new_task.task_type = t['task_type']
                new_task.start_time = t['start_time']

                new.append(new_task)
        return new

    def move_agents(self):
        return 0

    def compute_statistics(self):
        counters_dict = {}
        counters_dict[TaskState.PENDING] = 0
        counters_dict[TaskState.ASSIGNED] = 0
        counters_dict[TaskState.EXECUTED] = 0
        counters_dict[TaskState.COMPLETED] = 0
        for task in self.solver.tasks.values():
            counters_dict[task.task_state] += 1

        print("Pending tasks counter = ", counters_dict[TaskState.PENDING])
        print("Assigned tasks counter = ", counters_dict[TaskState.ASSIGNED])
        print("Executed tasks counter = ", counters_dict[TaskState.EXECUTED])
        print("Completed tasks counter = ", counters_dict[TaskState.COMPLETED])
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
