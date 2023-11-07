from msl.loadlib import LoadLibrary
from ctypes import *
from Simulations.classes import *

max_path_length = 1000


class AgentPath(Structure):
    _fields_ = ("length", c_int), ("x", c_int * max_path_length), ("y", c_int * max_path_length)


class LNS_Wrapper(object):
    # Load the external C++ DLL of LNS
    map_file_name = b"D:\GitHub\Multi-Agent_Pickup_and_Delivery\input_warehouse_delivery_stations_test.map"
    cpp = LoadLibrary(r'D:\GitHub\MAPF-LNS2\Release\lns.dll')
    map_string_buffer = create_string_buffer(map_file_name)
    default_time_limit = c_double(5)
    random_seed = 0
    number_of_agents = 0
    agents_keys = []

    @staticmethod
    def initialize(agents, shelves):
        LNS_Wrapper.agents_keys.clear()
        LNS_Wrapper.number_of_agents = len(agents)
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
        StartColArray = c_int * LNS_Wrapper.number_of_agents
        pointer_to_agents_start_col = pointer(StartColArray())
        StartRowArray = c_int * LNS_Wrapper.number_of_agents
        pointer_to_agents_start_row = pointer(StartRowArray())
        TargetColArray = c_int * LNS_Wrapper.number_of_agents
        pointer_to_agents_target_col = pointer(TargetColArray())
        TargetRowArray = c_int * LNS_Wrapper.number_of_agents
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
            LNS_Wrapper.agents_keys.append(agent['name'])
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
        LNS_Wrapper.cpp.lib.init_lns(LNS_Wrapper.map_string_buffer, LNS_Wrapper.number_of_agents, pointer_to_agents_start_col,
                         pointer_to_agents_start_row, pointer_to_agents_target_col,
                         pointer_to_agents_target_row, pointer_to_busy_agents_ids,
                         n_busy_agents, pointer_to_shelves_col, pointer_to_shelves_row, n_shelves)

    @staticmethod
    def run(time_limit=None):
        AgentsPathsArray = AgentPath * LNS_Wrapper.number_of_agents
        pointer_to_agents_paths = pointer(AgentsPathsArray())

        # Calling INVOKE_LNS method from external C++ DLL
        if time_limit is None:
            return_value = LNS_Wrapper.cpp.lib.invoke_lns(LNS_Wrapper.default_time_limit,
                                                          LNS_Wrapper.random_seed, byref(pointer_to_agents_paths))
        else:
            return_value = LNS_Wrapper.cpp.lib.invoke_lns(c_double(time_limit), LNS_Wrapper.random_seed,
                                                   byref(pointer_to_agents_paths))

        if return_value != 1 : # LNS Failure
            return False, None
        else:   # LNS Success
            agent_result_paths = {}
            for agent_id in range(LNS_Wrapper.number_of_agents):
                agent_path = []
                for state_id in range(pointer_to_agents_paths.contents[agent_id].length):
                    new_state = {}
                    x = pointer_to_agents_paths.contents[agent_id].x[state_id]
                    y = pointer_to_agents_paths.contents[agent_id].y[state_id]
                    new_state['t'] = state_id
                    new_state['x'] = x
                    new_state['y'] = y
                    agent_path.append(new_state)
                agent_name = LNS_Wrapper.agents_keys[agent_id]
                agent_result_paths[agent_name] = agent_path

            return True, agent_result_paths