import random
from Simulations.central_algorithm import Task, TaskState
import math
from enum import Enum

class GenerationScheme(Enum):
    RANDOM = 0
    FROMFILE = 1

class TaskGenerator(object):
    def __init__(self, starts, goals, sampled_starts=None, sampled_goals=None):
        self.next_start_sample_index = -1
        self.next_goal_sample_index = -1
        self.n_samples = 10000
        self.starts = starts
        self.goals = goals
        self.taken_squares = set()
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
            next_start = random.choice(self.starts)
            next_goal = random.choice(self.goals)
            self.next_starts.append(tuple([next_start[0], next_start[1]]))
            self.next_goals.append(tuple([next_goal[0], next_goal[1]]))

    def get_next_samples(self):
        self.next_start_sample_index += 1
        next_start = self.next_starts[self.next_start_sample_index]
        while next_start in self.taken_squares:
            self.next_start_sample_index += 1
            next_start = self.next_starts[self.next_start_sample_index]

        self.next_goal_sample_index += 1
        next_goal = self.next_goals[self.next_goal_sample_index]
        while next_goal in self.taken_squares:
            self.next_goal_sample_index += 1
            next_goal = self.next_goals[self.next_goal_sample_index]

        return next_start, next_goal

    def generate_new_tasks(self, current_tasks, n_tasks, current_time):
        new_tasks = []
        n_tasks_so_far = len(current_tasks)
        self.taken_squares.clear()
        for task in current_tasks.values():
            if task.task_state == TaskState.PENDING or task.task_state == TaskState.ASSIGNED:
                self.taken_squares.add(tuple([task.start_pos[0], task.start_pos[1]]))
                self.taken_squares.add(tuple([task.goal_pos[0], task.goal_pos[1]]))
            if task.task_state == TaskState.EXECUTED:
                self.taken_squares.add(tuple([task.goal_pos[0], task.goal_pos[1]]))

        for i in range(n_tasks):
            next_start, next_goal = self.get_next_samples()

            self.taken_squares.add(next_start)
            self.taken_squares.add(next_goal)

            new_task = Task()
            new_task.task_name = 'task' + str(n_tasks_so_far + i)
            new_task.start_pos = next_start
            new_task.goal_pos = next_goal
            new_task.task_state = TaskState.PENDING
            new_task.task_type = 0
            new_task.start_time = int(current_time)
            new_tasks.append(new_task)

        return new_tasks
