import argparse
import yaml
import json
import os
from Simulation.TP_with_recovery import TokenPassingRecovery
from Simulation.central_algorithm import Central
import RoothPath
from Simulation.tasks_and_delays_maker import *
from Simulation.simulation import Simulation
import subprocess
import sys

if __name__ == '__main__':

    # Loading command lines arguments
    #random.seed(1234)
    parser = argparse.ArgumentParser()
    parser.add_argument('-k', help='Robustness parameter for k-TP', default=1, type=int)
    parser.add_argument('-p', help='Robustness parameter for p-TP', default=None, type=float)
    parser.add_argument('-pd', help='Expected probability of an agent of being delayed at any time step (p-TP)',
                        default=0.02, type=float)
    parser.add_argument('-p_iter', help='Number of times a new path can be recalculated if the one calculated '
                                        'before exceeds the probability threshold (p-TP)',
                        default=1, type=int)
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true')

    args = parser.parse_args()

    # Reading config file
    with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))

    # Setting output file
    args.output = os.path.join(RoothPath.get_root(), 'output.yaml')

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']

    solver_freq = 1   # every cycle
    cycles_since_last_solver_run = solver_freq

    if args.not_rand:
        # Old fixed tasks and delays
        tasks = param['tasks']
    else:
        # Generate random tasks and delays
        tasks = gen_tasks_and_delays(agents, param['map']['start_locations'], param['map']['goal_locations'],
                                             param['n_tasks'], param['task_freq'])
    param['tasks'] = tasks
    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Instaniate a Solver object
    solver = Central(agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter=args.a_star_max_iter)

    # Instantiate a Simulation object
    simulation = Simulation(tasks, agents, solver)

    new_tasks = []
    while not simulation.simulation_ended():

        # Gathering new tasks introduced in the current time step
        new_tasks_buffer = simulation.get_new_tasks()
        for t in new_tasks_buffer:
            new_tasks.append(t)

        # Check if is time to invoke the solver
        if cycles_since_last_solver_run == solver_freq:
            solver.timestep(simulation.time, new_tasks)
            new_tasks.clear()
            cycles_since_last_solver_run = 0
            simulation.actual_paths = solver.paths

        cycles_since_last_solver_run = cycles_since_last_solver_run + 1

        # Moving agents according to their current plans
        simulation.move_agents()

        # Keeping record of benchmark statistics
        simulation.compute_statistics()

        # Incrementing simulation time by 1
        simulation.time = simulation.time + 1

        for task_name in solver.tasks.keys():
            solver.completed_tasks_times[task_name] = 0

        break

    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)
    # output = {'schedule': simulation.actual_paths, 'cost': cost,
    #           'completed_tasks_times': tp.get_completed_tasks_times(),
    #           'n_replans': tp.get_n_replans()}
    output = {'schedule': simulation.actual_paths, 'cost': cost,
              'completed_tasks_times': solver.completed_tasks_times,
              'n_replans': 0}
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

    create = [sys.executable, '-m', 'Utils.Visualization.visualize', '-slow_factor', str(args.slow_factor)]
    subprocess.call(create)
