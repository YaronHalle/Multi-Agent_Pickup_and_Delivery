"""
Python implementation of Token Passing algorithms to solve MAPD problems with delays
author: Giacomo Lodigiani (@Lodz97)
"""
from math import fabs
from Simulations.classes import *
import numpy as np
import random
from copy import deepcopy
import scipy.optimize
from Utils.Visualization.visualize import *
from Simulations.CBS.cbs import CBS, Environment
from collections import defaultdict


def is_agent_at_goal(agent_record):
    return agent_record['current_pos'][0] == agent_record['goal'][0] and \
        agent_record['current_pos'][1] == agent_record['goal'][1]


class Central(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter=4000):
        # Assert that number of agents doesn't exceed number of possible non-task endpoints
        if len(agents) > len(non_task_endpoints):
            print('There are more agents than non task endpoints, instance is not well-formed.')
            exit(1)
        self.agents = agents
        self.agents_dict = {}
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
        self.unassigned_agents = []
        for a in self.agents:
            self.path_ends.add(tuple(a['start']))

        for agent_record in self.agents:
            agent_name = agent_record['name']
            self.agents_dict[agent_name] = agent_record
            # Every agent is initially set to FREE state
            #agent_record['state'] = AgentState.FREE

            # Initializing an index to the agent's path current position
            agent_record['current_path_index'] = 0

            # Determining whether agent state is IDLE or FREE according to its initial position
            agent_pos = tuple([agent_record['current_pos'][0], agent_record['current_pos'][1]])
            if agent_pos in self.non_task_endpoints:
                # Agent rests at some non-task endpoint and should be referred as IDLE
                agent_record['state'] = AgentState.IDLE
                #self.free_non_task_endpoints.remove(agent_record['current_pos'])
                #self.occupied_non_task_endpoints.append(agent_record['current_pos'])
                self.free_non_task_endpoints.remove(agent_pos)
                self.occupied_non_task_endpoints.append(agent_pos)
            else:
                # Agent is not populating any endpoint and thus is FREE
                agent_record['state'] = AgentState.FREE

    def get_idle_agents(self):
        agents = {}
        for name, path in self.token['agents'].items():
            if len(path) == 1:
                agents[name] = path
        return agents

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
        # Computing cost of free agents to endpoints assuming no agent-agent collisions
        agent2task_cost = {}
        for agent_name in self.unassigned_agents:
            for endpoint in self.free_non_task_endpoints:
                agent = self.agents_dict[agent_name]
                if agent_name not in agent2task_cost:
                    agent2task_cost[agent_name] = {}
                agent['goal'] = endpoint
                env = Environment(self.dimensions, [agent], self.obstacles, None, self.a_star_max_iter)
                path = env.a_star.search(agent['name'])
                agent2task_cost[agent['name']][endpoint] = len(path)
                del agent['goal']
                del env

        # Populating cost_mat, rows are agents and columns are endpoints
        cost_mat = []
        for agent_name in agent2task_cost:
            for endpoint in agent2task_cost[agent_name]:
                cost_mat.append(agent2task_cost[agent_name][endpoint])
        n_agents = len(self.unassigned_agents)
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
            # Reseting agent's trajectory index since a path is about to re-computed
            agent_record['current_path_index'] = 0

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
                agent['current_pos'] = tuple([agent_trajectory[traj_index]['x'],
                                              agent_trajectory[traj_index]['y']])
            else:
                print('Warning: trying to move agent to non existing trajectory element')
            # Updating 'start' property so future CBS searches will take into account
            # the agent's current position
            agent['start'][0] = agent['current_pos'][0]
            agent['start'][1] = agent['current_pos'][1]
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Updating status for all agents that have reached their destinations.
            # Note that ENROUTE->BUSY transition is not handled here, but rather in
            # TimeStep() method
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if is_agent_at_goal(agent):
                # Updating status for agent's state transition FREE -> IDLE
                if agent['state'] == AgentState.FREE:
                    agent['state'] = AgentState.IDLE
                    self.free_non_task_endpoints.remove(agent['goal'])
                    del agent['goal']
                    del self.paths[agent_name]

                # Updating status for agent's state transition BUSY -> FREE
                if agent['state'] == AgentState.BUSY:
                    agent['state'] = AgentState.FREE
                    task_name = agent['task_name']
                    completed_task_record = self.tasks[task_name]
                    completed_task_record.task_state = TaskState.COMPLETED
                    completed_task_record.finish_time = time
                    del agent['goal']
                    del agent['task_name']
                    del self.agents_to_tasks[agent_name]
                    del self.tasks_to_agents[task_name]

    def time_step(self, time):
        agents_for_assignment = []
        agents_for_path_planning = []
        tasks_to_assign = []
        self.unassigned_agents.clear()
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Exiting if no new tasks were introduced since the last algorithm step
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # if len(new_tasks) == 0:
        #     return
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Collecting new tasks released since last algorithm step
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # for t in new_tasks:
        #     self.tasks[t.task_name] = t
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which agents should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for agent in self.agents:
            # Checking if ENROUTE agents have reached their pickup location and become BUSY
            if agent['state'] == AgentState.ENROUTE and is_agent_at_goal(agent):
                # Setting the task's delivery location as the agent's next goal
                task_goal = self.tasks[agent['task_name']].goal_pos
                agent['goal'] = task_goal

                # Updating the assigned task status
                self.tasks[agent['task_name']].task_state = TaskState.EXECUTED

                # Marking the agent for path planning determination
                agents_for_path_planning.append(agent)

                # Updating the agent's state to BUSY
                agent['state'] = AgentState.BUSY

            # Adding non-BUSY agents to potential task/endpoint assignment.
            elif agent['state'] == AgentState.BUSY:
                agents_for_path_planning.append(agent)
            else:
                agents_for_assignment.append(agent)
                agents_for_path_planning.append(agent)
                self.unassigned_agents.append(agent['name'])
            # elif agent['state'] != AgentState.BUSY:
            #     agents_for_assignment.append(agent)
            #     agents_for_path_planning.append(agent)
            #     self.unassigned_agents.append(agent['name'])
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which tasks should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for task in self.tasks.values():
            if task.task_state == TaskState.PENDING or task.task_state == TaskState.ASSIGNED:
                tasks_to_assign.append(task)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Task Assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if len(tasks_to_assign) > 0:
            # Computing cost of agents to tasks start location assuming no agent-agent collisions
            agent2task_cost = {}
            for agent in agents_for_assignment:
                for task in tasks_to_assign:
                    if agent['name'] not in agent2task_cost:
                        agent2task_cost[agent['name']] = {}
                    agent['goal'] = task.start_pos
                    env = Environment(self.dimensions, [agent], self.obstacles, None, self.a_star_max_iter)
                    path = env.a_star.search(agent['name'])
                    agent2task_cost[agent['name']][task.task_name] = len(path)
                    del agent['goal']
                    del env

            # Populating cost_mat, rows are agents and columns are tasks
            cost_mat = []
            for agent_name in agent2task_cost:
                for task_name in agent2task_cost[agent_name]:
                    cost_mat.append(agent2task_cost[agent_name][task_name])
            n_agents = len(agents_for_assignment)
            n_tasks = len(tasks_to_assign)
            cost_ar = np.array(cost_mat).reshape((n_agents, n_tasks))

            # Computing optimal assignment using the Hungarian algorithm
            agent_id, task_id = scipy.optimize.linear_sum_assignment(cost_ar)

            # Updating the assigned and unassigned agents lists
            for task_i in range(len(task_id)):
                # Retrieving assigned task name and setting its status to ASSIGNED (in case it previously was PENDING)
                task_name = list(tasks_to_assign)[task_id[task_i]].task_name
                # Setting the new assigned task state to ASSIGNED
                self.tasks[task_name].task_state = TaskState.ASSIGNED

                # Updating the assigned agent goal and status
                assigned_agent_name = list(agent2task_cost.keys())[agent_id[task_i]]
                agent_record = self.agents_dict[assigned_agent_name]
                agent_record['goal'] = self.tasks[task_name].start_pos

                # Checking if agent was previously IDLE and therefore its endpoint should be released
                if agent_record['state'] == AgentState.IDLE:
                    agent_pos = tuple([agent_record['current_pos'][0], agent_record['current_pos'][1]])
                    self.free_non_task_endpoints.append(agent_pos)
                agent_record['state'] = AgentState.ENROUTE
                # Checking if the assigned agent was originally assigned to a different task. If so, the previous
                # task becomes PENDING.
                if 'task_name' in agent_record.keys():
                    prev_task_name = agent_record['task_name']
                    if task_name != prev_task_name and self.tasks_to_agents[prev_task_name] == assigned_agent_name:
                        self.tasks[prev_task_name].task_state = TaskState.PENDING
                        del self.tasks_to_agents[prev_task_name]
                agent_record['task_name'] = task_name
                self.agents_to_tasks[assigned_agent_name] = task_name

                # Reseting agent's trajectory index since a path is about to be re-computed
                agent_record['current_path_index'] = 0

                # Check if this task was previously assigned to a different former agent. If so, change
                # the former agent's status to FREE and make additional updates for correct bookkeeping.
                if task_name in self.tasks_to_agents and self.tasks_to_agents[task_name] != assigned_agent_name:
                    prev_assigned_agent_name = self.tasks_to_agents[task_name]
                    prev_agent_record = self.agents_dict[prev_assigned_agent_name]

                    prev_agent_record['state'] == AgentState.FREE
                    if 'goal' in prev_agent_record.keys():
                        print('Warning! found a goal property where there souldn\'t be')
                    del self.agents_to_tasks[prev_assigned_agent_name]
                    del self.tasks_to_agents[task_name]

                # Making sure task is filed in the task and agents mappings
                if task_name not in self.tasks_to_agents:
                    # Updating records of mapping between agents and tasks
                    self.agents_to_tasks[assigned_agent_name] = task_name
                    self.tasks_to_agents[task_name] = assigned_agent_name

                # Updating the tracking of unassigned agents for future endpoints assignment
                # agents_for_path_planning.append(agent)
                # self.assigned_agents.append(assigned_agent_name)
                self.unassigned_agents.remove(assigned_agent_name)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Assign non-task endpoints to the free agents that weren't assigned tasks
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if len(self.unassigned_agents) > 0:
            # Do not consider endpoints assignments for an IDLE agent (meaning it is already located in some endpoint)
            temp_unassigned_agents = deepcopy(self.unassigned_agents)
            for agent_name in temp_unassigned_agents:
                agent_record = self.agents_dict[agent_name]
                if agent_record['state'] == AgentState.IDLE:
                    self.unassigned_agents.remove(agent_name)
                    agents_for_path_planning.remove(agent_record)
            del temp_unassigned_agents

            # Check if there are any FREE agents left that need to be assigned to some endpoint
            if len(self.unassigned_agents) > 0:
                self.assign_non_task_endpoints_to_free_agents()
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Path Planning using CBS. Assigned task are planned towards their task
        # pickup location whereas free agents are planned towards their assigned
        # non-task endpoint location.
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Plotting for debug
        show_current_state(self.dimensions, self.obstacles, self.non_task_endpoints, self.agents, self.tasks, time)
        #show_current_state(dimensions, obstacles, non_task_endpoints, agents, solver.tasks, simulation.time))
        for agent_record in self.agents:
            print(agent_record['name'], ' is in state = ', agent_record['state'])

        # Run CBS search only if there are agents that need to path plan for
        if len(agents_for_path_planning) > 0:
            # Collecting all the agents that are currently moving and weren't designated for re-path planning. These
            # agents would be considered as "moving obstacles" to avoid during the following CBS search.
            moving_obstacles = None  # self.get_currently_moving_agents(agents_for_path_planning)

            env = Environment(self.dimensions, agents_for_path_planning, self.obstacles, moving_obstacles, self.a_star_max_iter)
            cbs = CBS(env)
            mapf_solution = cbs.search()

            if len(mapf_solution) == 0:
                print('Warning! no CBS solution')

            # Updating paths only for agents with re-calculated paths
            for agent_record in agents_for_path_planning:
                agent_name = agent_record['name']
                self.paths[agent_name] = mapf_solution[agent_name]

            # Reseting each affected agent's path index
            for agent in agents_for_path_planning:
                agent['current_path_index'] = 0

            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Debug Printing
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            print('paths count = ', len(self.paths))
            if len(self.paths) == 0:
                print('Error: No CBS solution')
            for task_name in self.tasks_to_agents:
                print(task_name, ' --> ', self.tasks_to_agents[task_name])
            print('# free endpoints = ', len(self.free_non_task_endpoints))

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
                    # pos_index is subtracted to enforce current step to be regarded as "0" time in respect to rest of agents
                    # that are about to be re-planned.
                    t = agent_trajectory[k]['t'] - pos_index
                    temp_tuple = (x, y, t)
                    moving_obstacles[temp_tuple] = agent_record['name']
        return moving_obstacles

    def get_idle_agents(self):
        idle_agents = {}
        for agent_name in self.agents:
            if len(path) == 1:
                agents[name] = path
        return agents

    def time_forward(self):
        # Update completed tasks
        for agent_name in self.token['agents']:
            pos = self.simulation.actual_paths[agent_name][-1]
            if agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and self.token['agents_to_tasks'][agent_name][
                'task_name'] != 'safe_idle':
                self.token['completed_tasks'] = self.token['completed_tasks'] + 1
                self.token['completed_tasks_times'][
                    self.token['agents_to_tasks'][agent_name]['task_name']] = self.simulation.get_time()
                self.token['agents_to_tasks'].pop(agent_name)
            if agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and self.token['agents_to_tasks'][agent_name][
                'task_name'] == 'safe_idle':
                self.token['agents_to_tasks'].pop(agent_name)

        # Check delayed agents and agents affected by delays
        if self.new_recovery:
            self.token['delayed_agents'] = self.simulation.get_delayed_agents()
            for name in self.token['delayed_agents']:
                print('Agent', name, 'delayed or affected by delay!')
                path = self.token['agents'][name]
                self.token['n_replans'] = self.token['n_replans'] + 1
                self.update_ends(path[-1])
                if path[0] in self.non_task_endpoints:
                    self.token['occupied_non_task_endpoints'].add(tuple(path[0]))
                else:
                    self.token['path_ends'].add(tuple(path[0]))
                if name in self.token['agents_to_tasks']:
                    if self.token['agents_to_tasks'][name]['start'] not in path:
                        self.token['delayed_agents_to_reach_task_start'].append(name)
                self.token['agents'][name] = [path[0]]
        else:
            self.token['delayed_agents'] = []
            delayed_agents_pos = []
            for name, path in self.token['agents'].items():
                actual_state = self.simulation.get_actual_paths()[name][-1]
                if path[0] != [actual_state['x'], actual_state['y']]:
                    print('Agent', name, 'delayed!')
                    # self.token['n_replans'] = self.token['n_replans'] + 1
                    self.token['delayed_agents'].append(name)
                    delayed_agents_pos.append([actual_state['x'], actual_state['y']])
                    self.update_ends(path[-1])
                    pos = tuple([actual_state['x'], actual_state['y']])
                    if pos in self.non_task_endpoints:
                        self.token['occupied_non_task_endpoints'].add(pos)
                    else:
                        self.token['path_ends'].add(pos)
                    if self.token['agents_to_tasks'][name]['start'] not in path:
                        self.token['delayed_agents_to_reach_task_start'].append(name)
                    self.token['agents'][name] = [[actual_state['x'], actual_state['y']]]
            for name, path in self.token['agents'].items():
                if name not in self.token['delayed_agents']:
                    for i in range(len(path)):
                        if path[i] in delayed_agents_pos:
                            print('Agent', name, 'affected by delay!')
                            self.token['n_replans'] = self.token['n_replans'] + 1
                            self.update_ends(path[-1])
                            if path[0] in self.non_task_endpoints:
                                self.token['occupied_non_task_endpoints'].add(tuple(path[0]))
                            else:
                                self.token['path_ends'].add(tuple(path[0]))
                            if self.token['agents_to_tasks'][name]['start'] not in path:
                                self.token['delayed_agents_to_reach_task_start'].append(name)
                            self.token['agents'][name] = [path[0]]
                            break

        # TODO do this maybe only for old recovery
        # Check if somehow an agent path collides with an idle agent (may happen because of delays)
        if not self.new_recovery:
            for name, path in self.get_idle_agents().items():
                self.token['agent_at_end_path'].append(name)
                self.token['agent_at_end_path_pos'].append(path[0])
            for name, path in self.token['agents'].items():
                if name not in self.token['agent_at_end_path']:
                    for i in range(len(path)):
                        if path[i] in self.token['agent_at_end_path_pos']:
                            print('Agent', name, 'will impact end task agent, replanning...')
                            # self.update_ends(path[-1])
                            if path[0] in self.non_task_endpoints:
                                self.token['occupied_non_task_endpoints'].add(tuple(path[0]))
                            else:
                                self.token['path_ends'].add(tuple(path[0]))
                            # TODO check this rare keyerror
                            if self.token['agents_to_tasks'][name]['start'] not in path:
                                self.token['delayed_agents_to_reach_task_start'].append(name)
                            self.token['agents'][name] = [path[0]]
                            break
            self.token['agent_at_end_path'] = []
            self.token['agent_at_end_path_pos'] = []

        # Collect new tasks and assign them, if possible
        for t in self.simulation.get_new_tasks():
            self.token['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.token['start_tasks_times'][t['task_name']] = self.simulation.get_time()
        idle_agents = self.get_idle_agents()
        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            # agent_name = list(idle_agents.keys())[0]
            all_idle_agents = self.token['agents'].copy()
            all_idle_agents.pop(agent_name)
            all_delayed_agents = self.token['delayed_agents'].copy()
            if agent_name in all_delayed_agents:
                all_delayed_agents.remove(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            available_tasks = {}
            for task_name, task in self.token['tasks'].items():
                if tuple(task[0]) not in self.token['path_ends'].difference({tuple(agent_pos)}) and tuple(
                        task[1]) not in self.token['path_ends'].difference({tuple(agent_pos)}) \
                        and tuple(task[0]) not in self.get_agents_to_tasks_goals() and tuple(
                    task[1]) not in self.get_agents_to_tasks_goals():
                    available_tasks[task_name] = task
            if len(available_tasks) > 0 or agent_name in self.token['agents_to_tasks']:
                if agent_name in self.token['agents_to_tasks']:
                    closest_task_name = self.token['agents_to_tasks'][agent_name]['task_name']
                    if agent_name in self.token['delayed_agents_to_reach_task_start']:
                        closest_task = [self.token['agents'][agent_name][0],
                                        self.token['agents_to_tasks'][agent_name]['goal']]
                    else:
                        closest_task = [self.token['agents_to_tasks'][agent_name]['start'],
                                        self.token['agents_to_tasks'][agent_name]['goal']]
                else:
                    closest_task_name = self.get_closest_task_name(available_tasks, agent_pos)
                    closest_task = available_tasks[closest_task_name]
                moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
                idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), all_delayed_agents, 0)
                agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_task[0]}
                env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                                  moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
                cbs = CBS(env)
                path_to_task_start = self.search(cbs, agent_name, moving_obstacles_agents)
                if not path_to_task_start:
                    print("Solution not found to task start for agent", agent_name, " idling at current position...")
                    if len(self.token['delayed_agents']) == 0 and not self.token['prob_exceeded']:
                        print('Instance is not well-formed or a_star_max_iter is too low for this environment.')
                        self.deadlock_recovery(agent_name, agent_pos, all_idle_agents, all_delayed_agents, 4)
                        # exit(1)
                else:
                    print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
                    cost1 = env.compute_solution_cost(path_to_task_start)
                    # Use cost - 1 because idle cost is 1
                    moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], cost1 - 1)
                    idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), all_delayed_agents,
                                                                           cost1 - 1)
                    agent = {'name': agent_name, 'start': closest_task[0], 'goal': closest_task[1]}
                    env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                                      moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
                    cbs = CBS(env)
                    path_to_task_goal = self.search(cbs, agent_name, moving_obstacles_agents)
                    if not path_to_task_goal:
                        print("Solution not found to task goal for agent", agent_name, " idling at current position...")
                        if len(self.token['delayed_agents']) == 0 and not self.token['prob_exceeded']:
                            print('Instance is not well-formed  or a_star_max_iter is too low for this environment.')
                            self.deadlock_recovery(agent_name, agent_pos, all_idle_agents, all_delayed_agents, 4)
                            # exit(1)
                    else:
                        print("Solution found to task goal for agent", agent_name, " doing task...")
                        cost2 = env.compute_solution_cost(path_to_task_goal)
                        if agent_name not in self.token['agents_to_tasks']:
                            self.token['tasks'].pop(closest_task_name)
                            task = available_tasks.pop(closest_task_name)
                        else:
                            task = closest_task
                        if agent_name in self.token['delayed_agents_to_reach_task_start']:
                            self.token['delayed_agents_to_reach_task_start'].remove(agent_name)
                        last_step = path_to_task_goal[agent_name][-1]
                        self.update_ends(agent_pos)
                        self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                        self.token['agents_to_tasks'][agent_name] = {'task_name': closest_task_name, 'start': task[0],
                                                                     'goal': task[1], 'predicted_cost': cost1 + cost2}
                        self.token['agents'][agent_name] = []
                        for el in path_to_task_start[agent_name]:
                            self.token['agents'][agent_name].append([el['x'], el['y']])
                        # Don't repeat twice same step
                        self.token['agents'][agent_name] = self.token['agents'][agent_name][:-1]
                        for el in path_to_task_goal[agent_name]:
                            self.token['agents'][agent_name].append([el['x'], el['y']])
            elif self.check_safe_idle(agent_pos):
                print('No available tasks for agent', agent_name, ' idling at current position...')
            else:
                self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, all_delayed_agents)

        # Advance along paths in the token
        if not self.new_recovery:
            for name, path in self.token['agents'].items():
                if len(path) > 1:
                    self.token['agents'][name] = path[1:]
