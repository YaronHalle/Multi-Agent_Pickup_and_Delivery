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
        self.prev_baseline_total_cost = None
        self.prohibited_assignments = []
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

    def get_agents_names_for_task_splitting(self):
        # Collecting all BUSY agents that have already moved by at least a single step from the shelf's original position
        busy_agents_names = []
        for agent_record in self.baseline_solver.agents:
            if agent_record['state'] == AgentState.BUSY:
                task_name = agent_record['task_name']
                task = self.baseline_solver.tasks[task_name]
                if agent_record['current_pos'] != task.start_pos:
                    busy_agents_names.append(agent_record['name'])
        return busy_agents_names

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
        if len(agents_for_assignments) > 0:
            # There are available agents for assignment -> perform tasks assignment
            baseline_assignment_result = self.baseline_solver.assign_tasks(agents_for_assignments, tasks_to_assign,
                                                                           self.prohibited_assignments)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Compute baseline solution total cost (pickup costs + delivery costs)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # If all agents are BUSY then taking the previous computed solution and reducing the cost
        # of all EN-ROUTE and BUSY agents steps from the previous step
        if len(agents_for_assignments) == 0:
            baseline_total_cost = self.prev_baseline_total_cost
            for agent in self.baseline_solver.agents:
                if agent['state'] == AgentState.BUSY or agent['state'] == AgentState.ENROUTE:
                    baseline_total_cost -= 1
        else:
            baseline_total_cost = self.baseline_solver.compute_solution_total_cost(baseline_assignment_result)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Considering Task Splitting among BUSY agents
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # debug
        for agent_record in self.baseline_solver.agents:
            print(agent_record['name'], ' is in state = ', agent_record['state'])

        # Storing a copy of agents and tasks dictionaries for reverting changes
        # agents_copy = deepcopy(self.baseline_solver.agents)
        # agents_dict_copy = deepcopy(self.baseline_solver.agents_dict)
        # tasks_copy = deepcopy(self.baseline_solver.tasks)
        # tasks_to_agents_copy = deepcopy(self.baseline_solver.tasks_to_agents)
        # agents_to_tasks_copy = deepcopy(self.baseline_solver.agents_to_tasks)
        # free_non_task_endpoints_copy = deepcopy(self.baseline_solver.free_non_task_endpoints)
        agents_for_assignments_copy = deepcopy(agents_for_assignments)
        tasks_to_assign_copy = deepcopy(tasks_to_assign)
        new_prohibited_assignments = []  # List of lists([agent_name, task_name])
        agents_names_for_task_splitting = self.get_agents_names_for_task_splitting()
        better_solution_found = False
        commit_new_cbs_solution = False
        best_solution_cost_so_far = baseline_total_cost
        while len(agents_names_for_task_splitting) > 0:
            # Storing a copy of agents and tasks dictionaries for reverting changes
            agents_copy = deepcopy(self.baseline_solver.agents)
            tasks_copy = deepcopy(self.baseline_solver.tasks)

            # Creating a new solver instance for not damaging the data consistency of the real solver
            solver = ClassicMAPDSolver(agents_copy, self.baseline_solver.dimensions, self.baseline_solver.obstacles,
                                       self.baseline_solver.non_task_endpoints, self.baseline_solver.a_star_max_iter)

            agent_name = random.choice(agents_names_for_task_splitting)
            agent_record = solver.agents_dict[agent_name]

            # Setting the agent's task state back to PENDING (it was originally ASSIGNED)
            task_name = agent_record['task_name']
            #task_record = deepcopy(self.baseline_solver.tasks[task_name])
            task_record = tasks_copy[task_name]
            task_record.task_state = TaskState.PENDING

            # Setting the agent's state to FREE
            agent_record['state'] = AgentState.FREE
            del agent_record['task_name']
            del agent_record['goal']

            # Updating task's starting position since its shelf is about to be dropped off at the agent's current position
            task_record.start_pos = agent_record['current_pos']

            # Marking this task as type=1 (task affected by non-atomicity approach)
            task_record.task_type = 1

            # Adding this task back to tasks_to_assign list. Adding the agent to agents_for_assignment list
            tasks_to_assign_copy.append(task_record)
            #solver.add_tasks(deepcopy(tasks_to_assign_copy))
            solver.add_tasks(tasks_copy.values())
            solver.tasks_to_agents = deepcopy(self.baseline_solver.tasks_to_agents)
            solver.agents_to_tasks = deepcopy(self.baseline_solver.agents_to_tasks)
            agents_for_assignments_copy.append(agent_record)

            # Updating agents_to_tasks and tasks_to_agents mappings
            del solver.tasks_to_agents[task_name]
            del solver.agents_to_tasks[agent_name]

            # Preventing this agent from being assigned to its original task
            new_prohibited_assignments.append([agent_name, task_name])

            # Computing task assignment
            tentative_assignment_result = solver.assign_tasks(agents_for_assignments_copy, tasks_to_assign_copy,
                                                              self.prohibited_assignments + new_prohibited_assignments)
            tentative_total_cost = solver.compute_solution_total_cost(tentative_assignment_result)

            # debug printing
            print(agent_name, ' considered dropping shelf at position ', agent_record['current_pos'])
            print('\t Best cost so far = ', best_solution_cost_so_far, ', Tentative cost = ', tentative_total_cost)

            if tentative_total_cost < best_solution_cost_so_far:
                best_solution_cost_so_far = tentative_total_cost
                better_solution_found = True
            else:
                agents_for_assignments_copy.remove(agent_record)
                new_prohibited_assignments.remove([agent_name, task_name])
                tasks_to_assign_copy.remove(task_record)

            agents_names_for_task_splitting.remove(agent_name)
            del solver
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Preparing the CBS search execution after taking into account the
        # recommended task splitting from the previous section
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if better_solution_found:
            # Storing a copy of agents and tasks dictionaries for reverting changes
            agents_copy = deepcopy(self.baseline_solver.agents)
            tasks_copy = deepcopy(self.baseline_solver.tasks)
            tasks_to_assign_copy = deepcopy(tasks_to_assign)
            agents_for_assignments_copy = deepcopy(agents_for_assignments)
            tentative_prohibited_assignments = deepcopy(self.prohibited_assignments)

            # Creating a new solver instance for not damaging the data consistency of the real solver
            solver = ClassicMAPDSolver(agents_copy, self.baseline_solver.dimensions, self.baseline_solver.obstacles,
                                       self.baseline_solver.non_task_endpoints, self.baseline_solver.a_star_max_iter)
            solver.add_tasks(tasks_copy.values())
            solver.tasks_to_agents = deepcopy(self.baseline_solver.tasks_to_agents)
            solver.agents_to_tasks = deepcopy(self.baseline_solver.agents_to_tasks)

            for constraint in new_prohibited_assignments:  # List of lists([agent_name, task_name])
                agent_name = constraint[0]
                task_name = constraint[1]
                agent_record = solver.agents_dict[agent_name]

                # Setting the agent's task state back to PENDING (it was originally ASSIGNED)
                task_record = solver.tasks[task_name]
                task_record.task_state = TaskState.PENDING

                # Setting the agent's state to FREE
                agent_record['state'] = AgentState.FREE

                # Updating task's starting position since its shelf is about to be dropped off at the agent's current position
                task_record.start_pos = agent_record['current_pos']

                # Marking this task as type=1 (task affected by non-atomicity approach)
                task_record.task_type = 1

                # Adding this task back to tasks_to_assign list. Adding the agent to agents_for_assignment list
                tasks_to_assign_copy.append(task_record)
                agents_for_assignments_copy.append(agent_record)

                # Adding the new constraint to prohibitied assignments list
                tentative_prohibited_assignments.append(constraint)

            # Computing the heuristic total cost of recommended task splitting
            print('Test a heursitic better solution comprised of # drop-offs : ', len(new_prohibited_assignments))
            new_assignment_result = solver.assign_tasks(agents_for_assignments_copy, tasks_to_assign_copy, tentative_prohibited_assignments)
            new_total_cost = solver.compute_solution_total_cost(new_assignment_result)
            print('New assignment solution total cost is ', new_total_cost,'. Baseline cost = ',baseline_total_cost)
            if new_total_cost >= baseline_total_cost:
                better_solution_found = False

        # If the heuristic total cost improved, invoking a CBS to validate that a true cost improvement is evident
        if better_solution_found:
            # Computing the previous CBS path cost
            prev_validated_cbs_cost = self.baseline_solver.compute_prev_cbs_plan_cost()

            # Executing CBS to evaluate the true cost of the paths subject to task splitting
            solver.assign_non_task_endpoints_to_free_agents()
            tentative_agents_for_path_planning = solver.determine_agents_for_path_planning()
            tentative_mapf_solution = solver.agents_path_planning(tentative_agents_for_path_planning)
            if tentative_mapf_solution is not None:
                # Computing validated plan cost
                tentative_validated_cbs_cost = solver.compute_cbs_plan_cost(tentative_mapf_solution)
                if tentative_validated_cbs_cost < prev_validated_cbs_cost:
                    print('Tentative CBS - better solution found !')
                    print('\tPrev cost = ', prev_validated_cbs_cost, ', New cost = ', tentative_validated_cbs_cost)
                    commit_new_cbs_solution = True
                    baseline_total_cost = tentative_validated_cbs_cost
            else:
                print('Tentative CBS - no solution found')
                better_solution_found = False

        # Storing the updated baseline assignment result for potential usage in the next step in case all agents are BUSY
        self.prev_baseline_total_cost = baseline_total_cost

        if commit_new_cbs_solution is True:
            self.perform_commit(tentative_mapf_solution, solver.agents, solver.agents_dict, solver.agents_to_tasks,
                                    solver.tasks_to_agents, solver.tasks, tentative_prohibited_assignments)

            # # Updating paths for planned agents
            # for agent_name in tentative_mapf_solution.keys():
            #     agent_record = self.baseline_solver.agents_dict[agent_name]
            #     self.baseline_solver.paths[agent_name] = tentative_mapf_solution[agent_name]
            #
            #     # Resetting each affected agent's path index
            #     agent_record['current_path_index'] = 0
            #
            # # Marking the tasks that were induced due to task splitting
            # for assignment in tentative_prohibited_assignments:
            #     task_name = assignment[1]
            #     self.baseline_solver.tasks[task_name].task_type = 1
        else:
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
            if mapf_solution is None:
                print('Error: No CBS solution')
        for task_name in self.baseline_solver.tasks_to_agents:
            print(task_name, ' --> ', self.baseline_solver.tasks_to_agents[task_name])
        print('# free endpoints = ', len(self.baseline_solver.free_non_task_endpoints))

    def perform_commit(self, mapf_solution, agents, agents_dict, agents_to_tasks,
                                tasks_to_agents, tasks, prohibited_assignments):
        # Commit the agents' paths
        # commit agent_dict
        # commit agent_to_tasks
        # commit tasks_to_agents
        # commit tasks
        self.baseline_solver.paths = deepcopy(mapf_solution)
        self.baseline_solver.agents = deepcopy(agents)
        self.baseline_solver.agents_dict.clear()
        for agent_record in self.baseline_solver.agents:
            self.baseline_solver.agents_dict[agent_record['name']] = agent_record
        self.baseline_solver.agents_to_tasks = deepcopy(agents_to_tasks)
        self.baseline_solver.tasks_to_agents = deepcopy(tasks_to_agents)
        self.baseline_solver.tasks = deepcopy(tasks)
        self.prohibited_assignments = deepcopy(prohibited_assignments)

        # Resetting each affected agent's path index
        for agent_name in mapf_solution.keys():
            agent_record = self.baseline_solver.agents_dict[agent_name]
            agent_record['current_path_index'] = 0

        # Marking the tasks that were induced due to task splitting
        for assignment in prohibited_assignments:
            task_name = assignment[1]
            self.baseline_solver.tasks[task_name].task_type = 1