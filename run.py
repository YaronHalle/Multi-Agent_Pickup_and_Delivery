import argparse
import yaml
import json
import os
from Simulations.simulation import Simulation
from Simulations.central_algorithm import ClassicMAPDSolver, TaskState
from Simulations.MAPD_NAT_algorithm import NonAtomicSolver
import RoothPath
from Simulations.tasks_and_delays_maker import *
from Simulations.task_generator import *
from Simulations.CBS.cbs import CBS, Environment

import subprocess
import sys
from Utils.Visualization.visualize import *

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
                        default=1e7, type=int)
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
    simulation_end_time = 3000 #param['simulation_end_time']

    # Adding the current_pos field for each agent
    for agent in agents:
        agent['current_pos'] = tuple([agent['start'][0], agent['start'][1]])

    solver_freq = 1   # every cycle
    cycles_since_last_solver_run = solver_freq

    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)
    # ----------------------------------------------------------------------
    # Save data to JSON file
    # ----------------------------------------------------------------------
    if True:
        tg = TaskGenerator(param['map']['start_locations'], param['map']['goal_locations'])
        data = dict()
        data['agents'] = agents
        data['dimensions'] = dimensions
        data['obstacles'] = obstacles
        data['non_task_endpoints'] = non_task_endpoints
        data['sampled_starts_positions'] = tg.next_starts
        data['sampled_goals_positions'] = tg.next_goals
        with open("data_file.json", "w") as write_file:
            json.dump(data, write_file)

    # ----------------------------------------------------------------------
    # Loading data from JSON file
    # ----------------------------------------------------------------------
    if False:
        with open("data_file.json", "r") as read_file:
            data = json.load(read_file)
            agents = data['agents']
            dimensions = data['dimensions']
            obstacles = []
            for obstacle in data['obstacles']:
                obstacles.append(tuple([obstacle[0], obstacle[1]]))
            non_task_endpoints = []
            for endpoint in data['non_task_endpoints']:
                non_task_endpoints.append(tuple([endpoint[0], endpoint[1]]))
            sampled_starts_positions = []
            for next_start in data['sampled_starts_positions']:
                sampled_starts_positions.append(tuple([next_start[0], next_start[1]]))
            sampled_goals_positions = []
            for next_goal in data['sampled_goals_positions']:
                sampled_goals_positions.append(tuple([next_goal[0], next_goal[1]]))
            tg = TaskGenerator(param['map']['start_locations'], param['map']['goal_locations'],
                               sampled_starts_positions, sampled_goals_positions)

    # Instantiate a Solver object
    # solver = ClassicMAPDSolver(agents, dimensions, obstacles, non_task_endpoints, args.a_star_max_iter)
    solver = NonAtomicSolver(agents, dimensions, obstacles, non_task_endpoints, args.a_star_max_iter)

    # Instantiate a Simulation object
    simulation = Simulation(solver.get_tasks(), agents, solver)
    simulation.simulation_end_time = simulation_end_time
    new_tasks = []



    while not simulation.simulation_ended():
        print('---------------------- Time = ', simulation.time, ' ----------------------')

        # Gathering new tasks introduced in the current time step
        new_tasks_buffer = tg.generate_new_tasks(solver.get_agents(), solver.get_tasks(), 10, simulation.time)

        # new_tasks_buffer = []
        # t1 = Task()
        # t1.task_name = 'task1'
        # #t1.start_pos = tuple([27, 2])
        # t1.goal_pos = tuple([24, 2])
        # t1.task_state = TaskState.EXECUTED
        # solver.baseline_solver.agents_dict['agent1']['state'] = AgentState.BUSY
        # solver.baseline_solver.agents_dict['agent1']['task_name'] = 'task1'
        # solver.baseline_solver.agents_dict['agent1']['goal'] = t1.goal_pos
        # solver.baseline_solver.agents_to_tasks['agent1'] = 'task1'
        # solver.baseline_solver.tasks_to_agents['task1'] = 'agent1'
        #
        # t2 = Task()
        # t2.task_name = 'task2'
        # #t2.start_pos = tuple([5, 2])
        # t2.goal_pos = tuple([6, 2])
        # t2.task_state = TaskState.EXECUTED
        # solver.baseline_solver.agents_dict['agent2']['state'] = AgentState.BUSY
        # solver.baseline_solver.agents_dict['agent2']['task_name'] = 'task2'
        # solver.baseline_solver.agents_dict['agent2']['goal'] = t2.goal_pos
        # solver.baseline_solver.agents_to_tasks['agent2'] = 'task2'
        # solver.baseline_solver.tasks_to_agents['task2'] = 'agent2'
        #
        # t3 = Task()
        # t3.task_name = 'task3'
        # t3.start_pos = tuple([2, 2])
        # t3.goal_pos = tuple([0, 1])
        # t3.task_state = TaskState.PENDING
        #
        # new_tasks_buffer.append(t1)
        # new_tasks_buffer.append(t2)
        # new_tasks_buffer.append(t3)
        #
        # solver.baseline_solver.update_shelves_locations()
        # env = Environment(solver.baseline_solver.dimensions, solver.baseline_solver.agents, solver.baseline_solver.obstacles,
        #                   [], solver.baseline_solver.shelves_locations, solver.baseline_solver.a_star_max_iter)
        # cbs = CBS(env)
        # mapf_solution = cbs.search()
        # [0,1], [27,2], [4,2], [2,2]

        # for t in new_tasks_buffer:
        #     solver.tasks[t.task_name] = t
        solver.add_tasks(new_tasks_buffer)

        # Check if it is time to invoke the solver
        if cycles_since_last_solver_run == solver_freq:
            solver.time_step(simulation.time)
            new_tasks.clear()
            cycles_since_last_solver_run = 0
            # simulation.actual_paths = solver.paths

        show_current_state(dimensions, obstacles, non_task_endpoints, solver.get_agents(), solver.get_tasks(), simulation.time)

        cycles_since_last_solver_run = cycles_since_last_solver_run + 1

        # Moving agents according to their current plans (if plans exist)
        solver.move_agents(simulation.time)

        # Keeping record of benchmark statistics
        simulation.compute_statistics()

        # Incrementing simulation time by 1
        simulation.time = simulation.time + 1

        # for task_name in solver.tasks.keys():
        #     solver.completed_tasks_times[task_name] = 15
    # if len(simulation.actual_paths) == 0:
    #     print('Warning: simulation.actual_paths is empty')
    print('---------------------- End of Simulation  ----------------------')

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
