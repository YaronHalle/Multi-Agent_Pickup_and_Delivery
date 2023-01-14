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
        self.statistics[self.time] = {}
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Counting tasks according to task's states
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        tasks_counters = {TaskState.PENDING.value: 0, TaskState.ASSIGNED.value: 0, TaskState.EXECUTED.value: 0, TaskState.COMPLETED.value: 0}
        for task in self.solver.tasks.values():
            tasks_counters[task.task_state.value] += 1
        # tasks_counters = {TaskState['PENDING'].value: 0, TaskState.ASSIGNED: 0, TaskState.EXECUTED: 0,
        #                       TaskState.COMPLETED: 0}
        # for task in self.solver.tasks.values():
        #     tasks_counters[TaskState[task.task_state].value] += 1

        self.statistics[self.time]['tasks_counters'] = tasks_counters
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Computing throughput
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if self.time > 0:
            throughput = self.statistics[self.time]['tasks_counters'][TaskState.COMPLETED.value] - \
                        self.statistics[self.time - 1]['tasks_counters'][TaskState.COMPLETED.value]
        else:
            throughput = 0
        self.statistics[self.time]['current_step_throughput'] = throughput
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Computing average throughput
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        throughput_sum = 0
        for record in self.statistics.values():
            throughput_sum += record['current_step_throughput']
        N_samples = len( self.statistics)
        if N_samples > 0:
            avg_throughput = throughput_sum / N_samples
        else:
            avg_throughput = None
        self.statistics[self.time]['avg_throughput'] = avg_throughput
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Computing average service time
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        N_samples = 0
        service_time_sum = 0
        for task in self.solver.tasks.values():
            if task.task_state == TaskState.COMPLETED:
                task_service_time = task.finish_time - task.start_time
                service_time_sum += task_service_time
                N_samples += 1
        if N_samples > 0:
            avg_service_time = service_time_sum / N_samples
        else:
            avg_service_time = None
        self.statistics[self.time]['avg_service_time'] = avg_service_time
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Computing average delay time
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        delay_time_sum = 0
        for task in self.solver.tasks.values():
            if task.task_state == TaskState.PENDING or task.task_state == TaskState.ASSIGNED:
                task.delay_time += 1
            delay_time_sum += task.delay_time
        if len(self.solver.tasks) > 0:
            avg_delay_time = delay_time_sum / len(self.solver.tasks)
        else:
            avg_delay_time = None
        self.statistics[self.time]['avg_delay_time'] = avg_delay_time
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Appending to JSON stats file
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        stats_filename = "stats_file.json"
        if self.time == 0:
            # Creating the JSON file for the first time
            with open(stats_filename, "w") as write_file:
                json.dump(self.statistics[self.time], write_file)
        else:
            with open(stats_filename, "r") as read_file:
                data = json.load(read_file)
            data[self.time] = self.statistics[self.time]
            with open(stats_filename, "w") as write_file:
                json.dump(data, write_file)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Debug Printing
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        print("Pending tasks counter = ", tasks_counters[TaskState.PENDING.value])
        print("Assigned tasks counter = ", tasks_counters[TaskState.ASSIGNED.value])
        print("Executed tasks counter = ", tasks_counters[TaskState.EXECUTED.value])
        print("Completed tasks counter = ", tasks_counters[TaskState.COMPLETED.value])
        print("Current step throughput = ", throughput)
        print("Average throughput so far = ", avg_throughput)
        print("Average delay time so far = ", avg_delay_time)
        print("Average service time so far = ", avg_service_time)
