from math import fabs
from Simulations.classes import *
import numpy as np
import random
from copy import deepcopy
import scipy.optimize
from Utils.Visualization.visualize import *
from Simulations.CBS.cbs import CBS, Environment
from collections import defaultdict
from Simulations.central_algorithm import ClassicMAPDSolver, TaskState, AgentState

class NonAtomicSolver(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter=1e6):

        self.baseline_solver = ClassicMAPDSolver(agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter)

        # self.agents = self.baseline_solver.agents
        # self.tasks = self.baseline_solver.tasks
        # self.agents_dict = {}
        # self.dimensions = dimensions
        # self.obstacles = set(obstacles)
        # self.non_task_endpoints = deepcopy(non_task_endpoints)
        # self.occupied_non_task_endpoints = []
        # self.free_non_task_endpoints = deepcopy(self.non_task_endpoints)
        # self.a_star_max_iter = a_star_max_iter
        # self.agents_to_tasks = {}
        # self.tasks_to_agents = {}
        # self.completed_tasks = 0
        # self.n_replans = 0
        # self.path_ends = set()
        # self.agent_at_end_path = []
        # self.agent_at_end_path_pos = []
        # self.paths = {}
        # self.assigned_agents = []
        # self.unassigned_agents = []
        # for a in self.agents:
        #     self.path_ends.add(tuple(a['start']))
        #
        # for agent_record in self.agents:
        #     agent_name = agent_record['name']
        #     self.agents_dict[agent_name] = agent_record
        #
        #     # Initializing an index to the agent's path current position
        #     agent_record['current_path_index'] = 0
        #
        #     # Determining whether agent state is IDLE or FREE according to its initial position
        #     agent_pos = tuple([agent_record['current_pos'][0], agent_record['current_pos'][1]])
        #     if agent_pos in self.non_task_endpoints:
        #         # Agent rests at some non-task endpoint and should be referred as IDLE
        #         agent_record['state'] = AgentState.IDLE
        #         self.free_non_task_endpoints.remove(agent_pos)
        #         self.occupied_non_task_endpoints.append(agent_pos)
        #     else:
        #         # Agent is not populating any endpoint and thus is FREE
        #         agent_record['state'] = AgentState.FREE
    def get_agents(self):
        return self.baseline_solver.get_agents()

    def get_tasks(self):
        return self.baseline_solver.get_tasks()
    def add_tasks(self, new_tasks):
        self.baseline_solver.add_tasks(new_tasks)

    def move_agents(self, time):
        self.baseline_solver.move_agents(time)

    def get_potential_agents_for_task_splitting(self):
        busy_agents = []
        for agent_record in self.baseline_solver.agents:
            if agent_record['state'] == AgentState.BUSY:
                busy_agents.append(agent_record)
        return busy_agents

    def time_step(self, time):
        # self.unassigned_agents.clear()
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which agents should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        agents_for_assignments = self.baseline_solver.determine_agents_for_assignments()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which tasks should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        tasks_to_assign = self.baseline_solver.collect_tasks_for_assignment()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Classic "atomic" Task Assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        prohibited_assignments = []  # List of lists([agent_name, task_name])
        baseline_assignment_result = self.baseline_solver.assign_tasks(agents_for_assignments, tasks_to_assign, prohibited_assignments)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Compute baseline solution total cost (pickup costs + delivery costs)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        baseline_total_cost = self.baseline_solver.compute_solution_total_cost(baseline_assignment_result)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Considering Task Splitting among BUSY agents
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # debug
        for agent_record in self.baseline_solver.agents:
            print(agent_record['name'], ' is in state = ', agent_record['state'])

        agents_for_task_splitting = self.get_potential_agents_for_task_splitting()
        while len(agents_for_task_splitting) > 0:
            # Storing a copy of agents and tasks dictionaries for reverting changes
            agents_copy = deepcopy(self.baseline_solver.agents)
            agents_dict_copy = deepcopy(self.baseline_solver.agents_dict)
            tasks_copy = deepcopy(self.baseline_solver.tasks)
            tasks_to_agents_copy = deepcopy(self.baseline_solver.tasks_to_agents)
            agents_to_tasks_copy = deepcopy(self.baseline_solver.agents_to_tasks)
            free_non_task_endpoints_copy = deepcopy(self.baseline_solver.free_non_task_endpoints)

            # solver = ClassicMAPDSolver(agents_copy, self.baseline_solver.dimensions, self.baseline_solver.obstacles,
            #                            self.baseline_solver.non_task_endpoints, self.baseline_solver.a_star_max_iter)

            next_agent_for_test = random.choice(agents_for_task_splitting)
            agent_name = next_agent_for_test['name']

            # Setting the agent's task back to PENDING (it was originally ASSIGNED)
            task_name = next_agent_for_test['task_name']
            task_record = self.baseline_solver.tasks[task_name]
            self.baseline_solver.tasks[task_name].task_state = TaskState.PENDING

            # Setting the agent's state to FREE
            next_agent_for_test['state'] = AgentState.FREE

            # Updating task's starting position since its shelf is about to be dropped off at the agent's current position
            task_record.start_pos = next_agent_for_test['current_pos']

            # Adding this task back to tasks_to_assign list. Adding the agent to agents_for_assignment list
            tasks_to_assign.append(self.baseline_solver.tasks[task_name])
            agents_for_assignments.append(next_agent_for_test)

            # Preventing this agent from being assigned to its original task
            prohibited_assignments.append([agent_name, task_name])

            # Computing task assignment
            tentative_assignment_result = self.baseline_solver.assign_tasks(agents_for_assignments, tasks_to_assign, prohibited_assignments)
            tentative_total_cost = self.baseline_solver.compute_solution_total_cost(tentative_assignment_result)

            # debug printing
            print(agent_name,' considered dropping shelf at position ', next_agent_for_test['current_pos'])
            print('\t Baseline cost = ', baseline_total_cost, ', Tentative cost = ', tentative_total_cost)

            if tentative_total_cost < baseline_total_cost:
                baseline_total_cost = tentative_total_cost
                baseline_assignment_result = tentative_assignment_result
            else:
                agents_for_assignments.remove(next_agent_for_test)
                prohibited_assignments.remove([agent_name, task_name])
                self.baseline_solver.agents = deepcopy(agents_copy)
                self.baseline_solver.agents_dict = deepcopy(agents_dict_copy)
                self.baseline_solver.tasks = deepcopy(tasks_copy)
                self.baseline_solver.tasks_to_agents = deepcopy(tasks_to_agents_copy)
                self.baseline_solver.agents_to_tasks = deepcopy(agents_to_tasks_copy)
                self.baseline_solver.free_non_task_endpoints = deepcopy(free_non_task_endpoints_copy)

            agents_for_task_splitting.remove(next_agent_for_test)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Assign non-task endpoints to the free agents that weren't assigned tasks
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.baseline_solver.assign_non_task_endpoints_to_free_agents()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Collect agents list for path planning (non-IDLE agents)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        agents_for_path_planning = self.baseline_solver.determine_agents_for_path_planning()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Path Planning using CBS. Assigned task are planned towards their task
        # pickup location whereas free agents are planned towards their assigned
        # non-task endpoint location.
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Plotting for debug
        # show_current_state(self.dimensions, self.obstacles, self.non_task_endpoints, self.agents, self.tasks, time)
        # show_current_state(dimensions, obstacles, non_task_endpoints, agents, solver.tasks, simulation.time))
        for agent_record in self.baseline_solver.agents:
            print(agent_record['name'], ' is in state = ', agent_record['state'])

        # Run CBS search only if there are agents that need to path plan for
        if len(agents_for_path_planning) > 0:
            for agent_record in agents_for_path_planning:
                print(agent_record['name'], ' goal is ', agent_record['goal'])

            mapf_solution = self.baseline_solver.agents_path_planning(agents_for_path_planning)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Debug Printing
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if len(mapf_solution) == 0:
            print('Error: No CBS solution')
        for task_name in self.baseline_solver.tasks_to_agents:
            print(task_name, ' --> ', self.baseline_solver.tasks_to_agents[task_name])
        print('# free endpoints = ', len(self.baseline_solver.free_non_task_endpoints))