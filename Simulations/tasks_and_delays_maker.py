import random
import math

def gen_tasks_and_delays(agents, starts, goals, n_tasks, task_freq):
    arrival_time = 0
    tasks = []

    taken_squares = set()

    for i in range(n_tasks):
        # Get the next probability value from Uniform(0,1)
        p = random.random()
        # Plug it into the inverse of the CDF of Exponential(task_freq)
        inter_arrival_time = -math.log(1.0 - p) / task_freq
        # Add the inter-arrival time to the running sum
        arrival_time = arrival_time + inter_arrival_time
        # Generate task
        # tasks.append({'start_time': int(arrival_time), 'start': random.choice(starts), 'goal': random.choice(goals),
        #               'task_name': 'task' + str(i), 'task_type': 0})

        # TODO delete
        next_start = random.choice(starts)
        next_goal = random.choice(goals)

        while tuple([next_start[0],next_start[1]]) in taken_squares or \
                tuple([next_goal[0], next_goal[1]]) in taken_squares:
            next_start = random.choice(starts)
            next_goal = random.choice(goals)

        taken_squares.add(tuple([next_start[0],next_start[1]]))
        taken_squares.add(tuple([next_goal[0], next_goal[1]]))
        tasks.append({'start_time': int(arrival_time), 'start': next_start, 'goal': next_goal,
                      'task_name': 'task' + str(i), 'task_type': 0})
        # TODO delete up to here

    return tasks
