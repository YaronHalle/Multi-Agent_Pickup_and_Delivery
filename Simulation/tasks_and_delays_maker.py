import random
import math

def gen_tasks_and_delays(agents, starts, goals, n_tasks, task_freq):
    arrival_time = 0
    tasks = []

    for i in range(n_tasks):
        # Get the next probability value from Uniform(0,1)
        p = random.random()
        # Plug it into the inverse of the CDF of Exponential(task_freq)
        inter_arrival_time = -math.log(1.0 - p) / task_freq
        # Add the inter-arrival time to the running sum
        arrival_time = arrival_time + inter_arrival_time
        # Generate task
        tasks.append({'start_time': int(arrival_time), 'start': random.choice(starts), 'goal': random.choice(goals),
                      'task_name': 'task' + str(i), 'task_type': 0})


    return tasks
