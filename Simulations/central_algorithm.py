from math import fabs
from Simulations.classes import *
import numpy as np
import random
from copy import deepcopy
import scipy.optimize
from Utils.Visualization.visualize import *
from Simulations.CBS.cbs import CBS, Environment
from collections import defaultdict
import time
from Simulations.LNS_wrapper import *


def is_agent_at_goal(agent_record):
    # In case agent hasn't been assigned with a goal due to MAPF failure, making sure the function
    # returns 'False' and doesn't crash
    if 'goal' in agent_record:
        return agent_record['current_pos'][0] == agent_record['goal'][0] and \
            agent_record['current_pos'][1] == agent_record['goal'][1]
    else:
        return False


class ClassicMAPDSolver(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter=1e6, delivery_stations={}):
        # Assert that number of agents doesn't exceed number of possible non-task endpoints
        if len(agents) > len(non_task_endpoints):
            print('There are more agents than non task endpoints, instance is not well-formed.')
            exit(1)
        self.agents = agents
        self.agents_dict = {}
        # self.agents_for_assignment = []
        # self.agents_for_path_planning = []
        self.dimensions = dimensions
        self.obstacles = set(obstacles)
        self.non_task_endpoints = deepcopy(non_task_endpoints)
        self.occupied_non_task_endpoints = []
        self.free_non_task_endpoints = deepcopy(self.non_task_endpoints)
        self.tasks = {}
        self.a_star_max_iter = a_star_max_iter
        self.agents_to_tasks = {}
        self.tasks_to_agents = {}
        self.completed_tasks = 0
        self.n_replans = 0
        self.path_ends = set()
        self.agent_at_end_path = []
        self.agent_at_end_path_pos = []
        self.paths = {}
        self.assigned_agents = []
        # self.unassigned_agents = []
        self.shelves_locations = set()
        self.task_assign_cache = {}
        self.splitting_stats = {}
        self.occupancy_map = np.zeros(dimensions)
        self.delivery_stations = delivery_stations

        # Populating the obstacles in the occupancy map
        for obstacle in obstacles:
            self.occupancy_map[obstacle] = 1
        '''
        # Small Warehouse
        # self.LNS = LNS_Wrapper_Class(b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_small_yaron.map")
        # Small Warehouse for testing the new delivery stations mechanism
        self.LNS = LNS_Wrapper_Class(
            b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_delivery_stations_test.map")
        # Big Warehouse
        # self.LNS = LNS_Wrapper_Class(b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_big_random.map")
        '''

        for a in self.agents:
            self.path_ends.add(tuple(a['start']))

        for agent_record in self.agents:
            agent_name = agent_record['name']
            self.agents_dict[agent_name] = agent_record

            # Initializing an index to the agent's path current position
            agent_record['current_path_index'] = 0

            # Determining whether agent state is IDLE or FREE according to its initial position. This will be done
            # only if there's no 'goal' attribute since the solver might be instantiated with a previously determined
            # agents conditions we don't want to damage (for example, making an EN-ROUTE agent FREE).
            if 'goal' not in agent_record.keys():
                agent_pos = tuple([agent_record['current_pos'][0], agent_record['current_pos'][1]])
                self.occupancy_map[agent_pos] = 1
                if agent_pos in self.non_task_endpoints:
                    # Agent rests at some non-task endpoint and should be referred as IDLE
                    agent_record['state'] = AgentState.IDLE
                    self.free_non_task_endpoints.remove(agent_pos)
                    self.occupied_non_task_endpoints.append(agent_pos)
                else:
                    # Agent is not populating any endpoint and thus is FREE
                    agent_record['state'] = AgentState.FREE

    def update_delivery_stations(self, delivery_stations):
        self.delivery_stations = delivery_stations

    def get_agents(self):
        return self.agents

    def get_tasks(self):
        return self.tasks

    def add_tasks(self, new_tasks):
        for task in new_tasks:
            self.tasks[task.task_name] = task

    def admissible_heuristic(self, task_pos, agent_pos):
        return fabs(task_pos[0] - agent_pos[0]) + fabs(task_pos[1] - agent_pos[1])

    def get_closest_task_name(self, available_tasks, agent_pos):
        closest = random.choice(list(available_tasks.keys()))
        dist = self.admissible_heuristic(available_tasks[closest][0], agent_pos)
        for task_name, task in available_tasks.items():
            if self.admissible_heuristic(task[0], agent_pos) < dist:
                closest = task_name
        return closest

    def get_moving_obstacles_agents(self, agents, time_start):
        obstacles = {}
        for name, path in agents.items():
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    for j in range(1, self.k + 1):
                        if i - j >= time_start:
                            obstacles[(path[i][0], path[i][1], k - j)] = name
                        obstacles[(path[i][0], path[i][1], k + j)] = name
                    # Mark last element with negative time to later turn it into idle obstacle
                    if i == len(path) - 1:
                        obstacles[(path[i][0], path[i][1], -k)] = name
        return obstacles

    def get_idle_obstacles_agents(self, agents_paths, delayed_agents, time_start):
        obstacles = set()
        for path in agents_paths:
            if len(path) == 1:
                obstacles.add((path[0][0], path[0][1]))
            if 1 < len(path) <= time_start:
                obstacles.add((path[-1][0], path[-1][1]))
        for agent_name in delayed_agents:
            obstacles.add(tuple(self.token['agents'][agent_name][0]))
        return obstacles

    def check_safe_idle(self, agent_pos):
        for task_name, task in self.token['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        return True

    def get_closest_non_task_endpoint(self, agent_pos):
        dist = -1
        res = -1
        for endpoint in self.non_task_endpoints:
            if endpoint not in self.token['occupied_non_task_endpoints']:
                if dist == -1:
                    dist = self.admissible_heuristic(endpoint, agent_pos)
                    res = endpoint
                else:
                    tmp = self.admissible_heuristic(endpoint, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = endpoint
        if res == -1:
            print('Error in finding non-task endpoint, is instance well-formed?')
            exit(1)
        return res

    def update_ends(self, agent_pos):
        if tuple(agent_pos) in self.token['path_ends']:
            self.token['path_ends'].remove(tuple(agent_pos))
        elif tuple(agent_pos) in self.token['occupied_non_task_endpoints']:
            self.token['occupied_non_task_endpoints'].remove(tuple(agent_pos))

    def get_agents_to_tasks_goals(self):
        goals = set()
        for el in self.token['agents_to_tasks'].values():
            goals.add(tuple(el['goal']))
        return goals

    def get_agents_to_tasks_starts_goals(self):
        starts_goals = set()
        for el in self.token['agents_to_tasks'].values():
            starts_goals.add(tuple(el['goal']))
            starts_goals.add(tuple(el['start']))
        return starts_goals

    def get_completed_tasks(self):
        return self.token['completed_tasks']

    def get_completed_tasks_times(self):
        return self.token['completed_tasks_times']

    def get_n_replans(self):
        return self.token['n_replans']

    def get_token(self):
        return self.token

    def get_k(self):
        return self.k

    def get_replan_every_k_delays(self):
        return self.replan_every_k_delays

    def search(self, cbs, agent_name, moving_obstacles_agents):
        path = None
        if self.p_max == 1:
            path = cbs.search()
        else:
            self.token['prob_exceeded'] = False
            mk = MarkovChainsMaker(self.token['agents'], self.pd)
            for iter in range(self.p_iter):
                path = cbs.search()
                tmp = []
                if path and len(path[agent_name]) > 1:
                    for el in path[agent_name]:
                        tmp.append([el['x'], el['y']])
                    dic = mk.get_conflict_prob_given_path(tmp)
                    if dic['prob'] > self.p_max:
                        self.token['prob_exceeded'] = True
                        print('Conflict probablility to high (', dic['prob'], ') replanning...')
                        path = None
                        moving_obstacles_agents[(dic['pos_max_conf'][0], dic['pos_max_conf'][1], -1)] = agent_name
                    else:
                        break
        return path

    def assign_non_task_endpoints_to_free_agents(self):
        # Identifying which agents are unassigned and need to seek an endpoint
        # assigned_agents_names = [item[0] for item in assignments]
        # agents_for_path_planning_copy = []
        # self.unassigned_agents = []
        # for agent_record in self.agents_for_path_planning:
        #     if agent_record['state'] == AgentState.BUSY or agent_record['state'] == AgentState.ENROUTE:
        #         agents_for_path_planning_copy.append(agent_record)
        #     elif agent_record['state'] == AgentState.FREE:
        #         agents_for_path_planning_copy.append(agent_record)
        #         self.unassigned_agents.append(agent_record['name'])
        # self.agents_for_path_planning = agents_for_path_planning_copy

        # agents_for_path_planning_names = set([agent_record['name'] for agent_record in agents_for_path_planning])
        # all_agents_names = set([agent_record['name'] for agent_record in self.agents])
        # self.unassigned_agents = all_agents_names - agents_for_path_planning_names
        # if len(self.unassigned_agents) > 0:
        # Do not consider endpoints assignments for an IDLE agent (meaning it is already located in some endpoint)
        # temp_unassigned_agents = deepcopy(self.unassigned_agents)
        # for agent_name in temp_unassigned_agents:
        #     agent_record = self.agents_dict[agent_name]
        #     if agent_record['state'] == AgentState.IDLE:
        #         self.unassigned_agents.remove(agent_name)
        #         agents_for_path_planning.remove(agent_record)
        # del temp_unassigned_agents

        # Identifying the FREE agents that need to be assigned to endpoints
        unassigned_agents = []
        for agent in self.agents:
            if agent['state'] == AgentState.FREE:
                unassigned_agents.append(agent['name'])

        # Making sure at least one FREE agent left that is required to be assigned to some endpoint
        if len(unassigned_agents) == 0:
            return 0

        # Computing cost of free agents to endpoints assuming no agent-agent collisions
        agent2task_cost = {}
        for agent_name in unassigned_agents:
            for endpoint in self.free_non_task_endpoints:
                agent = self.agents_dict[agent_name]
                if agent_name not in agent2task_cost:
                    agent2task_cost[agent_name] = {}
                agent['goal'] = endpoint
                env = Environment(self.dimensions, [agent], self.obstacles, None, self.shelves_locations,
                                  self.a_star_max_iter)
                path = env.a_star.search(agent['name'])
                agent2task_cost[agent['name']][endpoint] = len(path)
                del agent['goal']
                del env

        # Populating cost_mat, rows are agents and columns are endpoints
        cost_mat = []
        for agent_name in agent2task_cost:
            for endpoint in agent2task_cost[agent_name]:
                cost_mat.append(agent2task_cost[agent_name][endpoint])
        n_agents = len(unassigned_agents)
        n_endpoints = len(self.free_non_task_endpoints)
        cost_ar = np.array(cost_mat).reshape((n_agents, n_endpoints))

        # Computing optimal assignment using the Hungarian algorithm
        agent_id, endpoint_id = scipy.optimize.linear_sum_assignment(cost_ar)

        currently_assigned_endpoints = set()

        # For each agent setting its goal location to be the assigned non-task endpoint
        for endpoint_i in range(len(endpoint_id)):
            endpoint = self.free_non_task_endpoints[endpoint_id[endpoint_i]]
            assigned_agent_name = list(agent2task_cost.keys())[agent_id[endpoint_i]]
            agent_record = self.agents_dict[assigned_agent_name]
            self.agents_dict[assigned_agent_name]['goal'] = endpoint
            self.agents_dict[assigned_agent_name]['state'] = AgentState.FREE
            currently_assigned_endpoints.add(endpoint)
            # Resetting agent's trajectory index since a path is about to be re-computed
            agent_record['current_path_index'] = 0

        return len(unassigned_agents)

    def get_enroute_and_busy_agents_for_path_planning(self):
        agents_for_path_planning = []
        for agent in self.agents:
            if agent['state'] == AgentState.ENROUTE or agent['state'] == AgentState.BUSY:
                agents_for_path_planning.append(agent)

        return agents_for_path_planning

    def determine_agents_for_path_planning(self):
        agents_for_path_planning = []
        for agent in self.agents:
            if agent['state'] != AgentState.IDLE:
                agents_for_path_planning.append(agent)

        return agents_for_path_planning

    def go_to_closest_non_task_endpoint(self, agent_name, agent_pos, all_idle_agents, all_delayed_agents):
        closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), all_delayed_agents, 0)
        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_non_task_endpoint}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                          a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_non_task_endpoint = self.search(cbs, agent_name, moving_obstacles_agents)
        if not path_to_non_task_endpoint:
            print("Solution to non-task endpoint not found for agent", agent_name, " instance is not well-formed.")
            self.deadlock_recovery(agent_name, agent_pos, all_idle_agents, all_delayed_agents, 4)
            # exit(1)
        else:
            print('No available task for agent', agent_name, ' moving to safe idling position...')
            self.update_ends(agent_pos)
            self.token['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))
            self.token['agents_to_tasks'][agent_name] = {'task_name': 'safe_idle', 'start': agent_pos,
                                                         'goal': closest_non_task_endpoint, 'predicted_cost': 0}
            self.token['agents'][agent_name] = []
            for el in path_to_non_task_endpoint[agent_name]:
                self.token['agents'][agent_name].append([el['x'], el['y']])

    def get_random_close_cell(self, agent_pos, r):
        while True:
            cell = (
                agent_pos[0] + random.choice(range(-r - 1, r + 1)), agent_pos[1] + random.choice(range(-r - 1, r + 1)))
            if cell not in self.obstacles and cell not in self.token['path_ends'] and \
                    cell not in self.token['occupied_non_task_endpoints'] \
                    and cell not in self.get_agents_to_tasks_goals() \
                    and 0 <= cell[0] < self.dimensions[0] and 0 <= cell[1] < self.dimensions[1]:
                return cell

    def deadlock_recovery(self, agent_name, agent_pos, all_idle_agents, all_delayed_agents, r):
        self.token['deadlock_count_per_agent'][agent_name] += 1
        if self.token['deadlock_count_per_agent'][agent_name] >= 5:
            self.token['deadlock_count_per_agent'][agent_name] = 0
            random_close_cell = self.get_random_close_cell(agent_pos, r)
            moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), all_delayed_agents, 0)
            agent = {'name': agent_name, 'start': agent_pos, 'goal': random_close_cell}
            env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                              a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path_to_non_task_endpoint = self.search(cbs, agent_name, moving_obstacles_agents)
            if not path_to_non_task_endpoint:
                print("No solution to deadlock recovery for agent", agent_name, " retrying later.")
            else:
                # Don't consider this a task, so don't add to agents_to_tasks
                print('Agent', agent_name, 'causing deadlock, moving to safer position...')
                self.update_ends(agent_pos)
                self.token['agents'][agent_name] = []
                for el in path_to_non_task_endpoint[agent_name]:
                    self.token['agents'][agent_name].append([el['x'], el['y']])

    def move_agents(self, time):
        # Looping over all agents in some arbitrary order and consider one by one
        for agent in self.agents:
            agent_name = agent['name']

            # Making sure that if agent is FREE or IDLE it has no task_name property left
            if agent['state'] == AgentState.IDLE or agent['state'] == AgentState.FREE:
                if 'task_name' in agent.keys():
                    del agent['task_name']

            # Nothing to do if agent is IDLE
            if agent['state'] == AgentState.IDLE:
                continue
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Moving every agent in one step ahead according to its plan
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            agent_trajectory = self.paths[agent_name]
            agent['current_path_index'] = agent['current_path_index'] + 1
            traj_index = agent['current_path_index']
            if traj_index < len(agent_trajectory):
                # Updating the occupancy map
                agent['current_pos'] = tuple([agent_trajectory[traj_index]['x'],
                                              agent_trajectory[traj_index]['y']])

                if agent['state'] == AgentState.BUSY:
                    task = self.tasks[self.agents_to_tasks[agent_name]]
                    task.current_pos = deepcopy(agent['current_pos'])
            else:
                print('Warning: trying to move agent to non existing trajectory element')
            # Updating 'start' property so future CBS searches will take into account
            # the agent's current position
            agent['start'][0] = agent['current_pos'][0]
            agent['start'][1] = agent['current_pos'][1]
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Updating status for all agents that have reached their destinations
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if is_agent_at_goal(agent):
                # Updating status for agent's state transition FREE -> IDLE
                if agent['state'] == AgentState.FREE:
                    agent['state'] = AgentState.IDLE
                    self.free_non_task_endpoints.remove(agent['goal'])
                    del agent['goal']
                    del self.paths[agent_name]
                    continue

                # Updating status for agent's state transition BUSY -> FREE
                task_record = self.tasks[agent['task_name']]
                if agent['state'] == AgentState.BUSY:
                    if task_record.task_phase == TaskPhase.DELIVERY2PICKUP:
                        agent['state'] = AgentState.FREE
                        task_name = agent['task_name']
                        completed_task_record = self.tasks[task_name]
                        completed_task_record.task_state = TaskState.COMPLETED
                        completed_task_record.finish_time = time
                        del agent['goal']
                        del agent['task_name']
                        del self.agents_to_tasks[agent_name]
                        del self.tasks_to_agents[task_name]

                # Updating status for agent's state transition EN-ROUTE -> BUSY
                if agent['state'] == AgentState.ENROUTE:
                    # Setting the task's delivery location as the agent's next goal
                    task_goal = self.tasks[agent['task_name']].current_destination
                    agent['goal'] = task_goal

                    # Updating the assigned task status
                    self.tasks[agent['task_name']].task_state = TaskState.EXECUTED

                    # Updating the agent's state to BUSY
                    agent['state'] = AgentState.BUSY

    def update_shelves_locations(self):
        self.shelves_locations.clear()
        for task in self.tasks.values():
            #if task.task_state != TaskState.COMPLETED:
            if task.task_state == TaskState.PENDING or task.task_state == TaskState.ASSIGNED:
                shelf = tuple([task.current_pos[0], task.current_pos[1]])
                self.shelves_locations.add(shelf)

    def collect_tasks_for_assignment(self):
        tasks_to_assign = []
        available_spots = {}
        for station in self.delivery_stations.values():
            available_spots[station.delivery_pos] = station.available_spots_count()

        for task in self.tasks.values():
            if task.task_state == TaskState.ASSIGNED:
                tasks_to_assign.append(task)
            elif task.task_state == TaskState.PENDING:
                delivery_pos = task.delivery_station.delivery_pos
                if available_spots[delivery_pos] > 0:
                    tasks_to_assign.append(task)
                    available_spots[delivery_pos] -= 1

        return tasks_to_assign

        '''
        tasks_to_assign = []
        for task in self.tasks.values():
            if task.task_state == TaskState.PENDING or task.task_state == TaskState.ASSIGNED \
                    and task.delivery_station.is_available():
                tasks_to_assign.append(task)
        return tasks_to_assign
        '''

    def determine_agents_for_assignments(self):
        agents_for_assignment = []
        # Scanning for non-busy agents for potential tasks assignments
        for agent in self.agents:
            if agent['state'] != AgentState.BUSY:
                agents_for_assignment.append(agent)

        return agents_for_assignment

    def assign_tasks(self, agents_for_assignments, tasks_to_assign, prohibited_assignments=[]):
        # t1 = time.time()
        agent_task_cost = []
        if len(tasks_to_assign) > 0:
            # Computing cost of agents to tasks start location assuming no agent-agent collisions
            agent2task_cost = {}
            for agent in agents_for_assignments:
                for task in tasks_to_assign:
                    if agent['name'] not in agent2task_cost:
                        agent2task_cost[agent['name']] = {}

                    # Making sure current agent -> task assignment is allowed, otherwise its cost is set to infinity to prevent it
                    if [agent['name'], task.task_name] in prohibited_assignments:
                        agent2task_cost[agent['name']][task.task_name] = float('inf')
                    else:
                        # Check if current assignment is already available in the cache memory
                        agent_task_tuple = tuple([agent['name'], task.task_name])
                        if agent_task_tuple in self.task_assign_cache:
                            agent2task_cost[agent['name']][task.task_name] = self.task_assign_cache[agent_task_tuple]
                        else:
                            # Estimating distances using Manhattan heuristics
                            agent_pos = agent['current_pos']

                            task_pos = task.current_pos
                            distance = abs(agent_pos[0] - task_pos[0]) + abs(agent_pos[1] - task_pos[1])
                            agent2task_cost[agent['name']][task.task_name] = distance
                            # Storing to cache memory
                            self.task_assign_cache[agent_task_tuple] = distance

                if 'goal' in agent.keys():
                    del agent['goal']
                # if 'task_name' in agent.keys():
                #     del agent['task_name']

            # Populating cost_mat, rows are agents and columns are tasks
            cost_mat = []
            for agent_name in agent2task_cost:
                for task_name in agent2task_cost[agent_name]:
                    cost_mat.append(agent2task_cost[agent_name][task_name])
            n_agents = len(agents_for_assignments)
            n_tasks = len(tasks_to_assign)
            cost_ar = np.array(cost_mat).reshape((n_agents, n_tasks))

            # Computing optimal assignment using the Hungarian algorithm
            agent_id, task_id = scipy.optimize.linear_sum_assignment(cost_ar)

            # Updating the assigned and unassigned agents lists
            for task_i in range(len(task_id)):
                # Retrieving assigned task name and setting its status to ASSIGNED (in case it previously was PENDING)
                task_name = list(tasks_to_assign)[task_id[task_i]].task_name
                task_record = self.tasks[task_name]
                assigned_agent_name = list(agent2task_cost.keys())[agent_id[task_i]]

                # Subscribing the task to its corresponding delivery station on first time assignment of agent-task
                if task_name not in self.tasks_to_agents:
                    # yaron
                    # task_record = task_record.delivery_station.subscribe_task(task_record)
                    if task_record.task_phase == TaskPhase.PICKUP2DELIVERY:
                        task_record.delivery_station.subscribe_task(task_name)

                # Setting the new assigned task state to ASSIGNED
                task_record.task_state = TaskState.ASSIGNED

                # Updating the assigned agent goal and status
                agent_record = self.agents_dict[assigned_agent_name]
                agent_record['goal'] = task_record.current_pos

                # Checking if agent was previously IDLE and therefore its endpoint should be released
                if agent_record['state'] == AgentState.IDLE:
                    agent_pos = tuple([agent_record['current_pos'][0], agent_record['current_pos'][1]])
                    self.free_non_task_endpoints.append(agent_pos)
                agent_record['state'] = AgentState.ENROUTE

                # Checking if the assigned agent was originally assigned to a different task. If so, the previous
                # task becomes PENDING.
                unsubscription_needed = False
                if 'task_name' in agent_record.keys():
                    prev_task_name = agent_record['task_name']
                    if task_name != prev_task_name and prev_task_name in self.tasks_to_agents and \
                            self.tasks_to_agents[prev_task_name] == assigned_agent_name:
                        self.tasks[prev_task_name].task_state = TaskState.PENDING
                        del self.tasks_to_agents[prev_task_name]
                        prev_task_record = self.tasks[prev_task_name]
                        unsubscription_needed = True
                        # The following line was commented since unsubscription can be only done after tasks_to_agents
                        # member is populated (else the progress_queue method will fail)
                        # prev_task_record = task_record.delivery_station.unsubscribe_task(prev_task_record)

                agent_record['task_name'] = task_name
                self.agents_to_tasks[assigned_agent_name] = task_name

                # Check if this task was previously assigned to a different former agent. If so, change
                # the former agent's status to FREE and make additional updates for correct bookkeeping.
                if task_name in self.tasks_to_agents and self.tasks_to_agents[task_name] != assigned_agent_name:
                    prev_assigned_agent_name = self.tasks_to_agents[task_name]
                    prev_agent_record = self.agents_dict[prev_assigned_agent_name]

                    # If there's no goal property in prev_agent_record it means the previous agent is not assigned with any task
                    # if 'goal' not in prev_agent_record.keys():
                    #     prev_agent_record['state'] = AgentState.FREE

                    # If there's no goal property in prev_agent_record it means the previous agent is not assigned with any task
                    prev_agent_record['state'] = AgentState.FREE
                    if 'goal' in prev_agent_record.keys():
                        del prev_agent_record['goal']
                    if 'task_name' in prev_agent_record.keys():
                        del prev_agent_record['task_name']
                    if prev_assigned_agent_name in self.agents_to_tasks:
                        del self.agents_to_tasks[prev_assigned_agent_name]
                    if task_name in self.tasks_to_agents:
                        del self.tasks_to_agents[task_name]

                # Making sure task is filed in the task and agents mappings
                if task_name not in self.tasks_to_agents:
                    # Updating records of mapping between agents and tasks
                    self.agents_to_tasks[assigned_agent_name] = task_name
                    self.tasks_to_agents[task_name] = assigned_agent_name

                if unsubscription_needed:
                    # yaron
                    # prev_task_record = prev_task_record.delivery_station.unsubscribe_task(prev_task_record)
                    if prev_task_record.task_phase == TaskPhase.PICKUP2DELIVERY:
                        prev_task_record.delivery_station.unsubscribe_task(prev_task_record.task_name)

                agent_task_cost.append(
                    [assigned_agent_name, task_name, agent2task_cost[assigned_agent_name][task_name]])

        # t2 = time.time()
        # print(f'Tasks assignments took {t2 - t1} [sec]')

        return agent_task_cost

    def compute_prev_cbs_plan_cost(self):
        prev_cost = self.compute_mapf_plan_cost(self.paths)

        # Reducing -1 step for each EN-ROUTE or BUSY agent due to a single step from previous cycle
        for agent in self.agents:
            if agent['state'] == AgentState.ENROUTE or agent['state'] == AgentState.BUSY:
                prev_cost -= 1

        return prev_cost

    def compute_mapf_plan_cost(self, mapf_solution):
        total_pickup_cost = 0
        total_delivery_cost = 0
        self.update_shelves_locations()

        for agent_name in mapf_solution:
            agent_record = self.agents_dict[agent_name]
            # Summing the pickup paths' cost of EN-ROUTE agents
            if agent_record['state'] == AgentState.ENROUTE:
                agent_path_index = agent_record['current_path_index']
                agent_path_length = len(mapf_solution[agent_name])
                total_pickup_cost += (agent_path_length - agent_path_index) - 1

                # Adding the heuristic cost from pickup to delivery destination.
                # The following section was commented during the integration of LNS to cut
                # the running times. ENROUTE agents will have to wait for reaching their pickup
                # location and only then the LNS will compute their path towards the delivery location.
                task_name = agent_record['task_name']
                task = self.tasks[task_name]
                tentative_agent = deepcopy(agent_record)
                tentative_agent['current_pos'] = task.current_pos
                tentative_agent['goal'] = task.delivery_pos

                env = Environment(self.dimensions, [tentative_agent], self.obstacles, None, self.shelves_locations,
                                  self.a_star_max_iter)
                path = env.a_star.search(tentative_agent['name'])
                if path is False:
                    print('Warning! A* path length is zero in compute_solution_total_cost method !')
                else:
                    total_delivery_cost += len(path)
                del tentative_agent
                del env

            # Summing the delivery paths' cost of BUSY agents
            if agent_record['state'] == AgentState.BUSY:
                agent_path_index = agent_record['current_path_index']
                agent_path_length = len(mapf_solution[agent_name])
                total_delivery_cost += (agent_path_length - agent_path_index) - 1

        return total_pickup_cost + total_delivery_cost

    def compute_solution_total_cost(self, assignment_result):
        # Summing the pickup paths' cost (EN-ROUTE agents)
        total_pickup_cost = 0
        for assignment in assignment_result:
            total_pickup_cost += assignment[2]

        # Summing the delivery paths' cost comprised of EN-ROUTE agents that haven't yet arrived to the pickup location
        # plus the trajectories of agents that have already picked up their shelves (=BUSY agents)
        total_delivery_cost = 0
        # Computing the EN-ROUTE agents planned paths from their pickup to delivery destinations:
        # Iterating through tasks and computing the path length from task start to finish using A* search assuming no
        # agents collisions
        for assignment in assignment_result:
            agent = deepcopy(self.agents_dict[assignment[0]])
            task = self.tasks[assignment[1]]

            agent['current_pos'] = task.current_pos
            agent['goal'] = task.delivery_pos

            env = Environment(self.dimensions, [agent], self.obstacles, None, self.shelves_locations,
                              self.a_star_max_iter)
            path = env.a_star.search(agent['name'])
            if path is False:
                print('Warning! A* path length is zero in compute_solution_total_cost method !')
            else:
                total_delivery_cost += len(path)
            del agent
            del env

        # Computing the BUSY agents paths' costs using the previous executed CBS plans
        for agent_record in self.agents:
            if agent_record['state'] == AgentState.BUSY:
                agent = deepcopy(agent_record)
                task_name = agent['task_name']
                task = self.tasks[task_name]
                agent['goal'] = task.delivery_pos

                env = Environment(self.dimensions, [agent], self.obstacles, None, self.shelves_locations,
                                  self.a_star_max_iter)
                path = env.a_star.search(agent['name'])
                if path is False:
                    print('Warning! A* path length is zero in compute_solution_total_cost method !')
                else:
                    total_delivery_cost += len(path)
                del agent
                del env

        return total_pickup_cost + total_delivery_cost

    def get_tasks_to_agents(self):
        return self.tasks_to_agents

    def get_agents_dict(self):
        return self.agents_dict

    def get_shelves_locations(self):
        return self.shelves_locations

    def agents_path_planning(self, agents_for_path_planning, time_limit=None):
        t1 = time.time()
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # First, computing paths for all agents according to their goal properties
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # t1 = time.time()
        self.update_shelves_locations()
        LNS_Wrapper.initialize(agents_for_path_planning, self.shelves_locations)
        # t2 = time.time()
        # print('\tLNS_INIT completed in ', t2-t1, ' [sec]')

        # t1 = time.time()
        lns_ok, first_phase_mapf_solution = LNS_Wrapper.run(time_limit)
        # t2 = time.time()
        # print('\tLNS_RUN completed in ', t2 - t1, ' [sec]')

        if not lns_ok:
            # print('***** Warning! No LNS solution found. Skipping planning ****')
            return None
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Secondly, computing a tentative path for all ENROUTE agents after they 
        # arrived at their pickup location and need to plan a path to their
        # delivery location
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        enroute_agents = []
        shelves_copy = deepcopy(self.shelves_locations)
        for agent in agents_for_path_planning:
            if agent['state'] == AgentState.ENROUTE:
                # Creating a copy of the agent for not damaging its goal and path properties
                agent_copy = deepcopy(agent)

                # Updating the agent copy's properties to simulate its arrival at the pickup location
                agent_copy['state'] = AgentState.BUSY
                task_name = agent_copy['task_name']
                task = self.tasks[task_name]
                agent_copy['current_pos'] = task.current_pos
                agent_copy['goal'] = task.delivery_pos

                # Removing the agent pickup shelf from the shelves list since they shelf is picked up
                shelves_copy.remove(task.current_pos)

                # Adding the modified ENROUTE agent to the list for path planning
                enroute_agents.append(agent_copy)

        # t1 = time.time()
        LNS_Wrapper.initialize(enroute_agents, shelves_copy)
        # t2 = time.time()
        # print('\tLNS_INIT completed in ', t2 - t1, ' [sec]')

        # t1 = time.time()
        lns_ok, second_phase_mapf_solution = LNS_Wrapper.run(time_limit)
        # t2 = time.time()
        # print('\tLNS_RUN completed in ', t2 - t1, ' [sec]')

        if not lns_ok:
            # print('***** Warning! No LNS solution found. Skipping planning ****')
            return None
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Both first and second phase planning are feasible, computing plan
        # costs (pickup+delivery paths' costs) and updating agents' paths
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        total_pickup_cost = 0
        total_delivery_cost = 0

        for agent_name in first_phase_mapf_solution:
            agent_record = self.agents_dict[agent_name]

            # Ignoring free agents (who are heading towards some endpoint)
            if agent_record['state'] == AgentState.FREE:
                continue

            task = self.tasks[self.agents_to_tasks[agent_name]]
            task_waiting_and_service_time = task.delivery_station.get_waiting_and_service_time(task)

            # Summing the pickup paths' cost of EN-ROUTE agents
            if agent_record['state'] == AgentState.ENROUTE:
                agent_path_index = agent_record['current_path_index']
                agent_path_length = len(first_phase_mapf_solution[agent_name]) - agent_path_index - 1
                total_pickup_cost += agent_path_length

                # Adding the delivery cost for enroute agents using the second phase LNS run. Length is multiplied
                # by 2 since the agent must take the pod to the delivery position and bring it back to the pickup position
                enroute_agent_path_from_pickup_to_delivery = len(second_phase_mapf_solution[agent_name])
                total_delivery_cost += max(agent_path_length + enroute_agent_path_from_pickup_to_delivery,
                                           task_waiting_and_service_time) + enroute_agent_path_from_pickup_to_delivery

            # Summing the delivery paths' cost of BUSY agents
            if agent_record['state'] == AgentState.BUSY:
                # Need to distinguish between two types of BUSY agents 1. Pickup to Delivery: LNS has computed the
                # trajectory from the agent's current position towards the delivery position. Need to add the
                # distance of brining back the pod from the delivery to the pickup position. 2. Delivery to Pickup:
                # LNS has computed the last trajectory segment of brining the pod from the delivery position back to
                # the pickup location. Thus, no need to add anything.
                agent_path_index = agent_record['current_path_index']
                agent_path_length = len(first_phase_mapf_solution[agent_name]) - agent_path_index - 1

                if task.task_phase == TaskPhase.PICKUP2DELIVERY:
                    # LNS has computed the path from current location to delivery position. Using Manhattan distance
                    # to estimate the path's length from the delivery to the pickup position
                    path_length_delivery_to_pickup = abs(task.delivery_pos[0] - task.pickup_pos[0]) + \
                                                     abs(task.delivery_pos[1] - task.pickup_pos[1])
                    total_delivery_cost += max(agent_path_length, task_waiting_and_service_time) + \
                                           path_length_delivery_to_pickup
                else:  # task.task_state == TaskState.DELIVERY2PICKUP:
                    total_delivery_cost += agent_path_length

        total_plan_cost = total_pickup_cost + total_delivery_cost

        # Updating paths only for agents from the first phase LNS run
        for agent_record in agents_for_path_planning:
            agent_name = agent_record['name']
            self.paths[agent_name] = first_phase_mapf_solution[agent_name]

        # Resetting each affected agent's path index
        for agent in agents_for_path_planning:
            agent['current_path_index'] = 0

        t2 = time.time()
        print('\tLNS completed in ', t2-t1, ' [sec]')

        return total_plan_cost

    def agents_path_planning_backup(self, agents_for_path_planning):
        # t1 = time.time()
        self.update_shelves_locations()
        LNS_Wrapper.initialize(agents_for_path_planning, self.shelves_locations)
        # t2 = time.time()
        # print('\tLNS_INIT completed in ', t2-t1, ' [sec]')

        # t1 = time.time()
        lns_ok, mapf_solution = LNS_Wrapper.run()
        # t2 = time.time()
        # print('\tLNS_RUN completed in ', t2 - t1, ' [sec]')

        if not lns_ok:
            print('***** Warning! No LNS solution found. Skipping planning ****')
            return None

        # Updating paths only for agents with re-calculated paths
        for agent_record in agents_for_path_planning:
            agent_name = agent_record['name']
            self.paths[agent_name] = mapf_solution[agent_name]

        # Resetting each affected agent's path index
        for agent in agents_for_path_planning:
            agent['current_path_index'] = 0

        return mapf_solution

    # TODO remove this old version of path planning method that uses CBS as basic planner
    def agents_path_planning_obs2(self, agents_for_path_planning):
        # Collecting all the agents that are currently moving and weren't designated for re-path planning. These
        # agents would be considered as "moving obstacles" to avoid during the following CBS search.
        moving_obstacles = None  # self.get_currently_moving_agents(agents_for_path_planning)

        # Updating shelves locations to avoid collisions between busy agents and other shelves (not carried by anyone)
        self.update_shelves_locations()

        # debug
        # for agent_record in agents_for_path_planning:
        #     print(agent_record['name'], ' goal is ', agent_record['goal'])

        env = Environment(self.dimensions, agents_for_path_planning, self.obstacles, moving_obstacles,
                          self.shelves_locations, self.a_star_max_iter)
        cbs = CBS(env)
        mapf_solution = cbs.search()

        if len(mapf_solution) == 0:
            print('***** Warning! No CBS solution found. Skipping planning ****')
            return

        # Updating paths only for agents with re-calculated paths
        for agent_record in agents_for_path_planning:
            agent_name = agent_record['name']
            self.paths[agent_name] = mapf_solution[agent_name]

        # Resetting each affected agent's path index
        for agent in agents_for_path_planning:
            agent['current_path_index'] = 0

        return mapf_solution

    def time_step(self, current_time):
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which agents should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        agents_for_assignment = self.determine_agents_for_assignments()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which tasks should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        tasks_to_assign = self.collect_tasks_for_assignment()
        # Resetting the task<->agent costs cache from previous time step
        self.task_assign_cache = {}
        self.assign_tasks(agents_for_assignment, tasks_to_assign)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Assign non-task endpoints to the free agents that weren't assigned tasks
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.assign_non_task_endpoints_to_free_agents()
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Path Planning using CBS. Assigned task are planned towards their task
        # pickup location whereas free agents are planned towards their assigned
        # non-task endpoint location.
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Plotting for debug
        # show_current_state(self.dimensions, self.obstacles, self.non_task_endpoints, self.agents, self.tasks, current_time)
        # for agent_record in self.agents:
        #     print(agent_record['name'], ' is in state = ', agent_record['state'])
        agents_for_path_planning = self.determine_agents_for_path_planning()

        # Run CBS search only if there are agents that need to path plan for
        if len(agents_for_path_planning) > 0:
            t1 = time.time()
            self.agents_path_planning(agents_for_path_planning)
            t2 = time.time()
            print('\tMAPF completed in ', t2 - t1, ' [sec]')

            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Debug Printing
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # print('paths count = ', len(self.paths))
            # if len(self.paths) == 0:
            #     print('Error: No CBS solution')
            # for task_name in self.tasks_to_agents:
            #     print(task_name, ' --> ', self.tasks_to_agents[task_name])
            # print('# free endpoints = ', len(self.free_non_task_endpoints))

    def get_currently_moving_agents(self, agents_for_path_planning):
        # Identify all agents that weren't considered for path planning and are currently moving (Non IDLE agents)
        moving_obstacles = {}
        for agent_record in self.agents:
            if agent_record in agents_for_path_planning:
                continue
            if agent_record['state'] != AgentState.IDLE:
                # Add all future agents path steps to the moving obstacles data structure
                agent_trajectory = self.paths[agent_record['name']]
                pos_index = agent_record['current_path_index']
                for k in range(pos_index, len(agent_trajectory)):
                    x = agent_trajectory[k]['x']
                    y = agent_trajectory[k]['y']
                    # pos_index is subtracted to enforce current step to be regarded as "0" time in respect to rest
                    # of agents that are about to be re-planned.
                    t = agent_trajectory[k]['t'] - pos_index
                    temp_tuple = (x, y, t)
                    moving_obstacles[temp_tuple] = agent_record['name']
        return moving_obstacles

    def remove_task_from_cache(self, task_name):
        for cache_tuple in deepcopy(self.task_assign_cache):
            if task_name == cache_tuple[1]:
                del self.task_assign_cache[cache_tuple]
