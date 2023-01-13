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


class NonAtomicSolver(object):
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

            # Initializing an index to the agent's path current position
            agent_record['current_path_index'] = 0

            # Determining whether agent state is IDLE or FREE according to its initial position
            agent_pos = tuple([agent_record['current_pos'][0], agent_record['current_pos'][1]])
            if agent_pos in self.non_task_endpoints:
                # Agent rests at some non-task endpoint and should be referred as IDLE
                agent_record['state'] = AgentState.IDLE
                self.free_non_task_endpoints.remove(agent_pos)
                self.occupied_non_task_endpoints.append(agent_pos)
            else:
                # Agent is not populating any endpoint and thus is FREE
                agent_record['state'] = AgentState.FREE

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
            # Resetting agent's trajectory index since a path is about to re-computed
            agent_record['current_path_index'] = 0

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
        '''
        Algorithm scheme:
        1. [CENTRAL] Identify ENROUTE agents that have reached their goal and now become BUSY. Add them for path planning list.
        2. [CENTRAL] List all IDLE/FREE/ASSIGNED agents for possible task assignment, collect all tasks that are either PENDING or ASSIGNED.
        3. [CENTRAL] Perform task assignment between tasks and agents for assignments. Compute total solution
        4. [NAT] Loop over all BUSY agents (=carries shelves).
        '''
        agents_for_assignment = []
        agents_for_path_planning = []
        tasks_to_assign = []
        self.unassigned_agents.clear()
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

                # Resetting agent's trajectory index since a path is about to be re-computed
                # agent_record['current_path_index'] = 0

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

            # Resetting each affected agent's path index
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
                    # pos_index is subtracted to enforce current step to be regarded as "0" time in respect to rest
                    # of agents that are about to be re-planned.
                    t = agent_trajectory[k]['t'] - pos_index
                    temp_tuple = (x, y, t)
                    moving_obstacles[temp_tuple] = agent_record['name']
        return moving_obstacles
