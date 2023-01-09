from enum import Enum

class AgentState(Enum):
    IDLE = 0
    FREE = 1
    ENROUTE = 2
    BUSY = 3

class TaskState(Enum):
    PENDING = 0
    ASSIGNED = 1
    EXECUTED = 2
    COMPLETED = 3

class Agent(object):
    def __init__(self):
        self.agent_name


class Task(object):
    def __init__(self):
        self.task_name = None
        self.start_pos = None
        self.goal_pos = None
        self.task_state = None
        self.task_type = None
        self.start_time = None
        self.finish_time = None

