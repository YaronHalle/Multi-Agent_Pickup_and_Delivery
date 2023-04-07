from msl.loadlib import LoadLibrary
from ctypes import *
from Simulations.classes import *

max_path_length = 1000


class AgentPath(Structure):
    _fields_ = ("length", c_int), ("x", c_int * max_path_length), ("y", c_int * max_path_length)


class LNS_Wrapper_Class(object):
    def __init__(self, map_file_name):
        # Load the external C++ DLL of LNS
        self.cpp = LoadLibrary(r'D:\GitHub\MAPF-LNS2\Release\lns.dll')
        #map_string_buffer = create_string_buffer(b"D:/GitHub/LNS_DLL_TESTER/random.map")
        self.map_string_buffer = create_string_buffer(map_file_name)
        self.default_time_limit = c_double(5)
        self.random_seed = 0
        self.number_of_agents = 0
        self.agents_keys = []


    def initialize(self, agents, shelves):
        self.agents_keys.clear()
        self.number_of_agents = len(agents)
        busy_agents_ids = []
        agent_id = 0
        for agent in agents:
            if agent['state'] == AgentState.BUSY:
                busy_agents_ids.append(agent_id)
            agent_id += 1
        n_busy_agents = len(busy_agents_ids)
        n_shelves = len(shelves)

        BusyAgentsArray = c_int * n_busy_agents
        pointer_to_busy_agents_ids = pointer(BusyAgentsArray())
        StartColArray = c_int * self.number_of_agents
        pointer_to_agents_start_col = pointer(StartColArray())
        StartRowArray = c_int * self.number_of_agents
        pointer_to_agents_start_row = pointer(StartRowArray())
        TargetColArray = c_int * self.number_of_agents
        pointer_to_agents_target_col = pointer(TargetColArray())
        TargetRowArray = c_int * self.number_of_agents
        pointer_to_agents_target_row = pointer(TargetRowArray())
        ShelvesRowArray = c_int * len(shelves)
        pointer_to_shelves_row = pointer(ShelvesRowArray())
        ShelvesColArray = c_int * len(shelves)
        pointer_to_shelves_col = pointer(ShelvesColArray())

        agent_id = 0
        for agent in agents:
            pointer_to_agents_start_col.contents[agent_id] = agent['current_pos'][0]
            pointer_to_agents_start_row.contents[agent_id] = agent['current_pos'][1]
            pointer_to_agents_target_col.contents[agent_id] = agent['goal'][0]
            pointer_to_agents_target_row.contents[agent_id] = agent['goal'][1]
            self.agents_keys.append(agent['name'])
            agent_id += 1

        shelf_id = 0
        for shelf in shelves:
            pointer_to_shelves_col.contents[shelf_id] = shelf[0]
            pointer_to_shelves_row.contents[shelf_id] = shelf[1]
            shelf_id += 1

        busy_ids = 0
        for agent_id in busy_agents_ids:
            pointer_to_busy_agents_ids.contents[busy_ids] = agent_id
            busy_ids += 1


        # INIT_LNS method invocation from external C++ DLL
        self.cpp.lib.init_lns(self.map_string_buffer, self.number_of_agents, pointer_to_agents_start_col,
                         pointer_to_agents_start_row, pointer_to_agents_target_col,
                         pointer_to_agents_target_row, pointer_to_busy_agents_ids,
                         n_busy_agents, pointer_to_shelves_col, pointer_to_shelves_row, n_shelves)

        '''
        void init_lns(const char* map_file_name, int number_of_agents, int agents_start_col[],
		int agents_start_row[], int agents_target_col[], int agents_target_row[], 
		int busy_agents_ids[], int n_busy_agents, int shelves_col[], int shelves_row[], int n_shelves);
        '''

    def run(self, time_limit=None):
        AgentsPathsArray = AgentPath * self.number_of_agents
        pointer_to_agents_paths = pointer(AgentsPathsArray())

        # Calling INVOKE_LNS method from external C++ DLL
        if time_limit is None:
            return_value = self.cpp.lib.invoke_lns(self.default_time_limit, self.random_seed, byref(pointer_to_agents_paths))
        else:
            return_value = self.cpp.lib.invoke_lns(c_double(time_limit), self.random_seed,
                                                   byref(pointer_to_agents_paths))

        if return_value != 1 : # LNS Failure
            return False, None
        else:   # LNS Success
            agent_result_paths = {}
            for agent_id in range(self.number_of_agents):
                agent_path = []
                for state_id in range(pointer_to_agents_paths.contents[agent_id].length):
                    new_state = {}
                    x = pointer_to_agents_paths.contents[agent_id].x[state_id]
                    y = pointer_to_agents_paths.contents[agent_id].y[state_id]
                    new_state['t'] = state_id
                    new_state['x'] = x
                    new_state['y'] = y
                    agent_path.append(new_state)
                agent_name = self.agents_keys[agent_id]
                agent_result_paths[agent_name] = agent_path

            return True, agent_result_paths
