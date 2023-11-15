import random
from Simulations.central_algorithm import *
import math
from enum import Enum

class GenerationScheme(Enum):
    RANDOM = 0
    FROMFILE = 1

class TaskGenerator(object):
    def __init__(self, starts, goals, delivery_stations, sampled_starts=None, sampled_goals=None):
        self.delivery_stations = delivery_stations
        self.next_start_sample_index = -1
        self.next_goal_sample_index = -1
        self.n_samples = 1000000
        #self.n_samples = 10
        self.starts = set()
        self.goals = set()
        self.taken_squares = set()
        for start in starts:
            self.starts.add(tuple([start[0], start[1]]))
        for goal in goals:
            self.goals.add(tuple([goal[0], goal[1]]))
        if sampled_starts is None:
            self.opmode = GenerationScheme.RANDOM
            self.next_starts = []
            self.next_goals = []
            self.generate_random_positions()
        else:
            self.opmode = GenerationScheme.FROMFILE
            self.next_starts = sampled_starts
            self.next_goals = sampled_goals

    def generate_random_positions(self):
        for i in range(self.n_samples):
            next_start = random.choice(list(self.starts))
            next_goal = random.choice(list(self.goals))
            self.next_starts.append(tuple([next_start[0], next_start[1]]))
            self.next_goals.append(tuple([next_goal[0], next_goal[1]]))

    def get_next_samples(self):
        if self.next_start_sample_index < len(self.next_starts) - 1:
            self.next_start_sample_index += 1
        else:
            self.next_start_sample_index = 0
        next_start = self.next_starts[self.next_start_sample_index]
        while next_start in self.taken_squares:
            if self.next_start_sample_index < len(self.next_starts)-1:
                self.next_start_sample_index += 1
            else:
                self.next_start_sample_index = 0
            next_start = self.next_starts[self.next_start_sample_index]
        if self.next_goal_sample_index < len(self.next_goals)-1:
            self.next_goal_sample_index += 1
        else:
            self.next_goal_sample_index = 0
        next_goal = self.next_goals[self.next_goal_sample_index]
        '''
        while next_goal in self.taken_squares:
            if self.next_goal_sample_index < len(self.next_goals) - 1:
                self.next_goal_sample_index += 1
            else:
                self.next_goal_sample_index = 0
            next_goal = self.next_goals[self.next_goal_sample_index]
        '''
        return next_start, next_goal

    def generate_new_tasks(self, agents, current_tasks, n_tasks, current_time):
        new_tasks = []
        n_tasks_so_far = len(current_tasks)
        self.taken_squares.clear()
        for task in current_tasks.values():
            if task.task_state != TaskState.COMPLETED:
                self.taken_squares.add(tuple([task.pickup_pos[0], task.pickup_pos[1]]))
                self.taken_squares.add(tuple([task.current_pos[0], task.current_pos[1]]))

        for agent in agents:
            current_pos = tuple([agent['current_pos'][0], agent['current_pos'][1]])
            self.taken_squares.add(current_pos)

        # Iterating through all delivery positions and removing deliveries whose queue is full
        '''
        for goal in self.goals:
            if not self.delivery_stations[goal].is_available():
                self.taken_squares.add(goal)
        '''

        for i in range(n_tasks):
            free_starts = self.starts - self.taken_squares
            #free_goals = self.goals - self.taken_squares
            free_goals = self.goals

            if len(free_starts) == 0:
                print('Warning: No more available task starting locations for generation!')
                break
            if len(free_goals) == 0:
                print('Warning: No more available task goal locations for generation!')
                break

            next_start, next_goal = self.get_next_samples()

            self.taken_squares.add(next_start)
            #self.taken_squares.add(next_goal)

            new_task = Task()
            new_task.task_name = 'task' + str(n_tasks_so_far + i)
            new_task.pickup_pos = next_start
            new_task.current_pos = next_start
            new_task.delivery_pos = next_goal
            new_task.task_state = TaskState.PENDING
            new_task.task_phase = TaskPhase.PICKUP2DELIVERY
            new_task.task_type = 0
            new_task.start_time = int(current_time)
            new_task.delay_time = 0
            new_task.delivery_station = self.delivery_stations[new_task.delivery_pos]
            new_tasks.append(new_task)

        return new_tasks