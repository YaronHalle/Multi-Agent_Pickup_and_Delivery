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
import time
import subprocess
import sys
from Utils.Visualization.visualize import *

# Creating a global scope object for the LNS path planner
# Small Warehouse
# self.LNS = LNS_Wrapper_Class(b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_small_yaron.map")
# Small Warehouse for testing the new delivery stations mechanism
# LNS = LNS_Wrapper_Class(
#     b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_delivery_stations_test.map")
# Big Warehouse
# LNS = LNS_Wrapper_Class(b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_big_random.map")

if __name__ == '__main__':

    # Loading command lines arguments
    random.seed(11111)
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
    simulation_end_time = 1000 #param['simulation_end_time']

    # Adding the current_pos field for each agent
    for agent in agents:
        agent['current_pos'] = tuple([agent['start'][0], agent['start'][1]])

    # Instantiate a Solver object
    # solver = ClassicMAPDSolver(agents, dimensions, obstacles, non_task_endpoints, args.a_star_max_iter)
    solver = NonAtomicSolver(agents, dimensions, obstacles, non_task_endpoints, args.a_star_max_iter)

    # Creating the delivery stations objects according to the YAML data
    delivery_stations = {}
    for i in range(len(param['map']['goal_locations'])):
        delivery_pos = tuple(param['map']['goal_locations'][i])
        waiting_locations = param['map']['waiting_locations'][i]
        new_delivery_station = DeliveryStation(solver, delivery_pos, waiting_locations)
        delivery_stations[delivery_pos] = new_delivery_station

    solver.update_delivery_stations(delivery_stations)

    solver_freq = 1   # every cycle
    cycles_since_last_solver_run = solver_freq

    # with open(args.param + config['visual_postfix'], 'w') as param_file:
    #     yaml.safe_dump(param, param_file)
    # ----------------------------------------------------------------------
    # Save data to JSON file
    # ----------------------------------------------------------------------
    if False:
        tg = TaskGenerator(param['map']['start_locations'], param['map']['goal_locations'], delivery_stations)
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
    if True:
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
                               delivery_stations, sampled_starts_positions, sampled_goals_positions)


    # Instantiate a Simulation object
    simulation = Simulation(solver.get_tasks(), agents, solver)
    simulation.simulation_end_time = simulation_end_time
    new_tasks = []

    while not simulation.simulation_ended():
        print('---------------------- Time = ', simulation.time, ' ----------------------')

        # Plotting
        show_current_state(dimensions, obstacles, non_task_endpoints, solver.get_agents(), solver.get_tasks(),
                           simulation.time)

        # Gathering new tasks introduced in the current time step
        new_tasks_buffer = tg.generate_new_tasks(solver.get_agents(), solver.get_tasks(), 20, simulation.time)

        solver.add_tasks(new_tasks_buffer)

        # Check if it is time to invoke the solver
        if cycles_since_last_solver_run == solver_freq:
            solver.time_step(simulation.time)
            new_tasks.clear()
            cycles_since_last_solver_run = 0

        cycles_since_last_solver_run = cycles_since_last_solver_run + 1

        # Moving agents according to their current plans (if plans exist)
        solver.move_agents(simulation.time)

        # Making a single time step for every delivery station management
        for station in delivery_stations.values():
            station.time_step()

            # Debug
            '''
            tasks_count = 0
            if station.current_processed_task is not None:
                tasks_count += 1
            tasks_count += len(station.tasks_queue)
            print('No. tasks in station ', station.delivery_pos, ' is ', tasks_count)
            '''

        # Keeping record of benchmark statistics
        simulation.compute_statistics()

        # Incrementing simulation time by 1
        simulation.time = simulation.time + 1

    print('---------------------- End of Simulation  ----------------------')
    '''
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
    '''