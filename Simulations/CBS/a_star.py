"""
AStar search
author: Ashwin Bose (@atb033)
author: Giacomo Lodigiani (@Lodz97)
"""
import heapq
from Simulations.classes import *
from itertools import count
from copy import deepcopy

class AStar:
    def __init__(self, env):
        self.agents = env.agents
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors
        self.max_iter = env.a_star_max_iter
        self.shelves_locations = env.shelves_locations
        self.iter = 0

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search
        """
        initial_state = self.agent_dict[agent_name]["start"]
        is_agent_busy = False
        for agent in self.agents:
            if agent['name'] == agent_name and agent['state'] == AgentState.BUSY:
                is_agent_busy = True
                break

        step_cost = 1

        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {}
        g_score[initial_state] = 0

        f_score = {}
        h_score = self.admissible_heuristic(initial_state, agent_name)
        f_score[initial_state] = h_score

        heap = []
        index = count(0)
        heapq.heappush(heap, (f_score[initial_state], h_score, next(index), initial_state))

        # debug
        # if self.iter == self.max_iter:
        #     print('Warning: A* search has exhausted!')

        while open_set and (self.max_iter == -1 or self.iter < self.max_iter):
            self.iter = self.iter + 1
            current = heapq.heappop(heap)[3]

            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                # Filtering out locations that collide with a shelf in case the agent is BUSY (already carries a shelf)
                if is_agent_busy and tuple([neighbor.location.x, neighbor.location.y]) in self.shelves_locations:
                    continue

                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                h_score = self.admissible_heuristic(neighbor, agent_name)
                f_score[neighbor] = g_score[neighbor] + h_score
                heapq.heappush(heap, (f_score[neighbor], h_score, next(index), neighbor))
        # print('Warning: No solution found by low level A* !')
        return False