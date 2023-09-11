import time
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
import colorama
from colorama import Fore, Back, Style

class NonAtomicSolver(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter=1e6):

        self.baseline_solver = ClassicMAPDSolver(agents, dimensions, obstacles, non_task_endpoints, a_star_max_iter)
        self.prev_baseline_total_cost = None
        self.prohibited_assignments = []
        self.splitting_stats = self.baseline_solver.splitting_stats
        self.agents_perm_table = set()
        self.short_lns_time_out = 1 # [sec]
        self.long_lns_time_out = 10 # [sec]

    def get_agents(self):
        return self.baseline_solver.get_agents()

    def get_tasks(self):
        return self.baseline_solver.get_tasks()
    def add_tasks(self, new_tasks):
        self.baseline_solver.add_tasks(new_tasks)

    def move_agents(self, current_time):
        self.baseline_solver.move_agents(current_time)

    def manhattan_distance(self, p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def get_agents_names_for_task_splitting(self):
        # Collecting all BUSY agents that have already moved by at least a single step from the shelf's original position
        busy_agents_names = []
        for agent_record in self.baseline_solver.agents:
            if agent_record['state'] == AgentState.BUSY:
                task_name = agent_record['task_name']
                task = self.baseline_solver.tasks[task_name]

                travelled_distance_from_pickup = \
                    self.manhattan_distance(agent_record['current_pos'], task.start_pos)
                if travelled_distance_from_pickup > 10:
                    busy_agents_names.append(agent_record['name'])

                # TODO replacing the single step distance to manhattan distance computation
                # if agent_record['current_pos'] != task.start_pos:
                #     busy_agents_names.append(agent_record['name'])
        return busy_agents_names

    def permutations_recursion(self, agents_flags, index_to_start, max_agents, used_agents):
        if used_agents == max_agents:
            return
        for i in range(index_to_start, len(agents_flags)):
            if agents_flags[i] == 0:
                new_agents_flags = deepcopy(agents_flags)
                new_agents_flags[i] = 1
                self.agents_perm_table.add(tuple(new_agents_flags))
                new_index_to_start = i + 1
                self.permutations_recursion(new_agents_flags, new_index_to_start, max_agents, used_agents + 1)

    def build_permutations_table(self, agents_names_for_task_splitting, max_agents):
        self.agents_perm_table.clear()
        agents_flags = [0] * len(agents_names_for_task_splitting)
        self.permutations_recursion(deepcopy(agents_flags), 0, max_agents, 0)
        perm_table = []
        for agent_flags in self.agents_perm_table:
            option = []
            for agent_id in range(len(agent_flags)):
                if agent_flags[agent_id] == 1:
                    option.append(agents_names_for_task_splitting[agent_id])
            perm_table.append(option)
        return perm_table

    def get_current_goal_locations(self):
        goal_locations = []
        for task in self.baseline_solver.tasks.values():
            if task.task_state != TaskState.COMPLETED:
                goal_locations.append(task.goal_pos)

        return goal_locations

    def time_step(self, current_time):
        # Creating a copy of the solver class for not damaging the data integrity in case no solution is found
        agents_copy = deepcopy(self.baseline_solver.agents)
        tasks_copy = deepcopy(self.baseline_solver.tasks)

        # Creating a new solver instance for not damaging the data consistency of the real solver
        solver_copy = ClassicMAPDSolver(agents_copy, self.baseline_solver.dimensions, self.baseline_solver.obstacles,
                                   self.baseline_solver.non_task_endpoints, self.baseline_solver.a_star_max_iter)

        solver_copy.add_tasks(tasks_copy.values())
        solver_copy.tasks_to_agents = deepcopy(self.baseline_solver.tasks_to_agents)
        solver_copy.agents_to_tasks = deepcopy(self.baseline_solver.agents_to_tasks)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Reseting the task->agents costs mapping cache
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.baseline_solver.task_assign_cache = {}

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which agents should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # agents_for_assignments = self.baseline_solver.determine_agents_for_assignments()
        agents_for_assignments = solver_copy.determine_agents_for_assignments()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Determine which tasks should be considered for assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # tasks_to_assign = self.baseline_solver.collect_tasks_for_assignment()
        tasks_to_assign = solver_copy.collect_tasks_for_assignment()

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Classic "atomic" Task Assignment
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        proceed_from_last_plan = True
        if len(agents_for_assignments) > 0:
            # t1 = time.time()
            baseline_assignment_result = solver_copy.assign_tasks(agents_for_assignments, tasks_to_assign,
                                                                           self.prohibited_assignments)
            # t2 = time.time()
            # print(f'Tasks assignments took {t2-t1} [sec]')
            solver_copy.assign_non_task_endpoints_to_free_agents()
            agents_for_path_planning = solver_copy.determine_agents_for_path_planning()

            # t1 = time.time()
            baseline_total_cost = solver_copy.agents_path_planning(agents_for_path_planning, self.long_lns_time_out)
            # t2 = time.time()
            # print(f'LNS took {t2 - t1} [sec]')

            if baseline_total_cost is not None:
                #  Solution found, committing
                _agents_for_assignments = self.baseline_solver.determine_agents_for_assignments()
                _tasks_to_assign = self.baseline_solver.collect_tasks_for_assignment()
                _baseline_assignment_result = self.baseline_solver.assign_tasks(_agents_for_assignments, _tasks_to_assign,
                                                                               self.prohibited_assignments)
                self.baseline_solver.assign_non_task_endpoints_to_free_agents()
                _agents_for_path_planning = self.baseline_solver.determine_agents_for_path_planning()
                _baseline_total_cost = self.baseline_solver.agents_path_planning(_agents_for_path_planning, self.long_lns_time_out)
                proceed_from_last_plan = False
            else:
                # No solution found, progressing with previously computed plan
                proceed_from_last_plan = True
                print(Fore.RED + ' Warning! at baseline solution computation no solution found!' + Fore.RESET)

        if proceed_from_last_plan:
            print(Fore.YELLOW + 'Proceeding from plan from previous cycle...' + Fore.RESET)
            baseline_total_cost = self.prev_baseline_total_cost
            for agent in self.baseline_solver.agents:
                if agent['state'] == AgentState.ENROUTE or agent['state'] == AgentState.BUSY:
                    baseline_total_cost -= 1
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Considering Task Splitting among BUSY agents
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # debug
        for agent_record in self.baseline_solver.agents:
            print(agent_record['name'], ' is in state = ', agent_record['state'])

        # TODO - change configuration between ATOMIC and NON-ATOMIC
        agents_names_for_task_splitting = self.get_agents_names_for_task_splitting()
        # agents_names_for_task_splitting = []

        better_solution_found = False
        commit_new_mapf_solution = False
        best_solution_cost_so_far = baseline_total_cost

        if len(agents_names_for_task_splitting) > 0:
            # Build the permutations table
            t1 = time.time()
            perm_table = self.build_permutations_table(agents_names_for_task_splitting, len(agents_names_for_task_splitting))
            goal_locations = self.get_current_goal_locations()
            t2 = time.time()
            print(f'Building permutations table took {t2-t1} [sec]')
            best_perm_id = None
            best_perm_cost = baseline_total_cost
            best_negative_assignments = None
            next_perm_to_report = 10

            # debug printing
            print(f'Starting task splitting testing. Baseline cost = {baseline_total_cost}, '
                  f'N_agents = {len(agents_names_for_task_splitting)} , {len(perm_table)} permutations...')

            task_assign_cache = {}

            for perm_iteraion in range(len(perm_table)):
                # print('Perm_Iteration = ', perm_iteraion)
                agents_for_assignments_copy = deepcopy(agents_for_assignments)
                new_prohibited_assignments = []  # List of lists([agent_name, task_name])
                tasks_to_assign_copy = deepcopy(tasks_to_assign)

                # Retrieving the next permutation for testing
                agents_names_to_test = perm_table[perm_iteraion]

                # Storing a copy of agents and tasks dictionaries for reverting changes
                agents_copy = deepcopy(self.baseline_solver.agents)
                tasks_copy = deepcopy(self.baseline_solver.tasks)

                # Creating a new solver instance for not damaging the data consistency of the real solver
                solver = ClassicMAPDSolver(agents_copy, self.baseline_solver.dimensions, self.baseline_solver.obstacles,
                                           self.baseline_solver.non_task_endpoints, self.baseline_solver.a_star_max_iter)

                solver.task_assign_cache = task_assign_cache

                for agent_name in agents_names_to_test:
                    agent_record = solver.agents_dict[agent_name]

                    # Testing that agent doesn't drop the shelf on a goal location of some other's tasks
                    agent_pos = agent_record['current_pos']
                    if agent_pos in goal_locations:
                        continue

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
                    solver.add_tasks(tasks_copy.values())
                    solver.tasks_to_agents = deepcopy(self.baseline_solver.tasks_to_agents)
                    solver.agents_to_tasks = deepcopy(self.baseline_solver.agents_to_tasks)
                    agents_for_assignments_copy.append(agent_record)

                    # Updating agents_to_tasks and tasks_to_agents mappings
                    del solver.tasks_to_agents[task_name]
                    del solver.agents_to_tasks[agent_name]

                    # Preventing this agent from being assigned to its original task
                    new_prohibited_assignments.append([agent_name, task_name])

                # Skip the path planning if no task dropping is considered
                if len(new_prohibited_assignments) == 0:
                    continue

                # Computing task assignment
                # t1 = time.time()
                tentative_assignment_result = solver.assign_tasks(agents_for_assignments_copy, tasks_to_assign_copy,
                                                                  self.prohibited_assignments + new_prohibited_assignments)
                # t2 = time.time()
                # print(f'Task assignment took {t2-t1} [sec]')

                # Invoking path planning using LNS
                # tentative_total_cost = solver.compute_solution_total_cost(tentative_assignment_result)

                tentative_agents_for_path_planning = solver.get_enroute_and_busy_agents_for_path_planning()
                # t1 = time.time()
                tentative_mapf_solution_cost = solver.agents_path_planning(tentative_agents_for_path_planning, self.short_lns_time_out)
                # t2 = time.time()
                # print(f'LNS took {t2-t1} [sec]')
                if tentative_mapf_solution_cost is not None:
                    t1 = time.time()
                    #perm_cost = solver.compute_mapf_plan_cost(tentative_mapf_solution)
                    perm_cost = tentative_mapf_solution_cost
                    t2 = time.time()
                    # print(f'Plan cost computation took {t2-t1} [sec]')
                    rational_improv = (baseline_total_cost - perm_cost) / baseline_total_cost*100
                    if perm_cost < best_perm_cost and rational_improv > 10:
                        print(Fore.GREEN + '\tTime=', current_time,' : Best cost so far = ', perm_cost, ', prev best cost = ', best_perm_cost, 'Improvement (wrt baseline) = ',
                              round(rational_improv), '%' + Fore.RESET)
                        print(Fore.GREEN + '\tAgents to drop shelves:' + Fore.RESET)
                        for elem in new_prohibited_assignments:
                            print(Fore.GREEN + '\t', elem[0], '' + Fore.RESET)
                        best_perm_cost = perm_cost
                        best_negative_assignments = deepcopy(new_prohibited_assignments)
                        better_solution_found = True

                del agents_for_assignments_copy
                del new_prohibited_assignments
                del tasks_to_assign_copy
                del solver

                # Reporting
                completed_perc = round((perm_iteraion + 1) / len(perm_table) * 100)
                if completed_perc > next_perm_to_report:
                    print(Fore.CYAN + f'Testing task splitting permutations: {completed_perc}%' + Fore.RESET)
                    next_perm_to_report += 10

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Invoking the tasks assignment and LNS MAPF execution after taking into
        # account the recommended task splitting from the previous section
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

            for constraint in best_negative_assignments:  # List of lists([agent_name, task_name])
                agent_name = constraint[0]
                task_name = constraint[1]
                agent_record = solver.agents_dict[agent_name]

                # Setting the agent's task state back to PENDING (it was originally ASSIGNED)
                task_record = solver.tasks[task_name]
                task_record.task_state = TaskState.PENDING

                # Since task's shelf position was updated, need to remove all agents->tasks costs that were
                # involved with this task (since they are no longer valid)
                solver.remove_task_from_cache(task_name)

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

            # Assigning tasks for the last time
            solver.assign_tasks(agents_for_assignments_copy, tasks_to_assign_copy,
                                                              self.prohibited_assignments + best_negative_assignments)
            self.baseline_solver.assign_non_task_endpoints_to_free_agents()
            best_agents_for_path_planning = solver.determine_agents_for_path_planning()
            #best_mapf_solution = solver.agents_path_planning(best_agents_for_path_planning)
            best_mapf_solution_cost = solver.agents_path_planning(best_agents_for_path_planning, self.short_lns_time_out)
            if best_mapf_solution_cost is not None:
                # best_mapf_solution_cost = solver.compute_mapf_plan_cost(best_mapf_solution)
                if best_mapf_solution_cost > best_perm_cost:
                    print('WARNING! RECOMMENDED MAPF COST HAS CHANGED!')
                    exit(-1)
                    commit_new_mapf_solution = False
                else:
                    commit_new_mapf_solution = True
            else:
                print('WARNING! EXPECTED THAT RECOMMENDED MAPF WILL BE FEASIBLE AND IT IS NOT!')

            # Saving statistics about the splitting performance
            rational_improv = (baseline_total_cost - best_mapf_solution_cost) / baseline_total_cost * 100
            self.baseline_solver.splitting_stats[current_time] = [len(best_negative_assignments), rational_improv]

        if commit_new_mapf_solution is True:
            self.perform_commit(solver, tentative_prohibited_assignments)

            # Storing the updated baseline assignment result for potential usage in the next step in case all agents are BUSY
            self.prev_baseline_total_cost = best_mapf_solution_cost

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
            # Storing the updated baseline assignment result for potential usage in the next step in case all agents are BUSY
            self.prev_baseline_total_cost = baseline_total_cost
            print(Fore.YELLOW + 'No better solution found due task splitting. Keeping original plan as is...' + Fore.RESET)
            '''
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Assign non-task endpoints to the free agents that weren't assigned tasks
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            n_assigned_free_agents = self.baseline_solver.assign_non_task_endpoints_to_free_agents()

            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Collect agents list for path planning (non-IDLE agents)
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if n_assigned_free_agents > 0 or len(agents_for_assignments) > 0:
                agents_for_path_planning = self.baseline_solver.determine_agents_for_path_planning()
            else:
                agents_for_path_planning = []

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

                # mapf_solution = self.baseline_solver.agents_path_planning(agents_for_path_planning)

                # Storing the updated baseline assignment result for potential usage in the next step in case all agents are BUSY
                # mapf_solution_cost = self.baseline_solver.compute_mapf_plan_cost(mapf_solution)
                mapf_solution_cost = self.baseline_solver.agents_path_planning(agents_for_path_planning)
                self.prev_baseline_total_cost = mapf_solution_cost

                # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                # Debug Printing
                # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if mapf_solution_cost is None:
                    print('Error: No CBS solution')
            '''

        for task_name in self.baseline_solver.tasks_to_agents:
            print(task_name, ' --> ', self.baseline_solver.tasks_to_agents[task_name])
        print('# free endpoints = ', len(self.baseline_solver.free_non_task_endpoints))

    def perform_commit(self, solver, prohibited_assignments):
        # Commit the agents' paths
        # commit agent_dict
        # commit agent_to_tasks
        # commit tasks_to_agents
        # commit tasks
        self.baseline_solver.paths = deepcopy(solver.paths)
        self.baseline_solver.agents = deepcopy(solver.agents)
        self.baseline_solver.agents_dict.clear()
        for agent_record in self.baseline_solver.agents:
            self.baseline_solver.agents_dict[agent_record['name']] = agent_record
        self.baseline_solver.agents_to_tasks = deepcopy(solver.agents_to_tasks)
        self.baseline_solver.tasks_to_agents = deepcopy(solver.tasks_to_agents)
        self.baseline_solver.tasks = deepcopy(solver.tasks)
        self.prohibited_assignments = deepcopy(prohibited_assignments)
        self.baseline_solver.task_assign_cache = deepcopy(solver.task_assign_cache)

        # Resetting each affected agent's path index
        for agent_name in solver.paths.keys():
            agent_record = self.baseline_solver.agents_dict[agent_name]
            agent_record['current_path_index'] = 0

        # Marking the tasks that were induced due to task splitting
        for assignment in prohibited_assignments:
            task_name = assignment[1]
            self.baseline_solver.tasks[task_name].task_type = 1

