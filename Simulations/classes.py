from enum import Enum, IntEnum
from copy import deepcopy
import numpy as np


class AgentState(IntEnum):
    IDLE = 0
    FREE = 1
    ENROUTE = 2
    BUSY = 3


class TaskState(IntEnum):
    PENDING = 0
    ASSIGNED = 1
    PICKUP2DELIVERY = 2
    DELIVERY2PICKUP = 3
    COMPLETED = 4


class Task(object):
    def __init__(self):
        self.task_name = None
        self.pickup_pos = None
        self.delivery_pos = None
        self.task_state = None
        self.task_type = None
        self.start_time = None
        self.delay_time = None
        self.finish_time = None
        self.current_pos = None
        self.current_destination = None
        self.delivery_station = None

class DeliveryStation(object):
    def __init__(self, solver, delivery_pos, waiting_locations, service_time=15):
        self.solver = solver
        self.delivery_pos = delivery_pos

        # Determining whether pods service time is constant or random
        self.is_random_service_time = False
        self.service_time = service_time

        self.queue_size = len(waiting_locations)
        self.waiting_locations = waiting_locations
        self.tasks_queue = []
        self.tasks_service_times = []
        self.next_waiting_index = 0
        self.time_to_finish_current_task = 0
        self.current_processed_task = None
        self.current_waiting_and_service_time = 0

    def get_waiting_and_service_time(self, task):
        """
        Returns the currently expected queue waiting time plus the service time for a specific task.
        :return: Expected waiting+service time ahead for the desired task.
        """
        total_time = self.time_to_finish_current_task
        for queue_task, service_time in zip(self.tasks_queue, self.tasks_service_times):
            total_time += service_time
            if queue_task == task:
                break

        return total_time

    def is_delivery_spot_free(self):
        """
        Checks whether the delivery spot itself is free.
        :return:  True/False
        """
        # Delivery spot is considered occupied in the following two situations:
        # (1) The delivery spot is vacant, but it is reserved for some en-route agent who's about to arrive
        # (2) The delivery spot is not reserved for anybody, but a previous agent has dropped its shelf in the delivery spot
        return (self.delivery_pos not in self.solver.shelves_locations) and self.current_processed_task is None

    def is_available(self):
        """
        Checks whether the delivery station can accept a newly assigned task to it.
        The delivery station cannot accept new tasks in two cases:
        [1] The delivery spot itself is either blocked by some dropped pod or it was already reserved for some enroute agent.
        [2] The queue is not fully occupied.
        :return: True/False
        """
        return self.is_delivery_spot_free() or (self.next_waiting_index is not None)

    def available_spots_count(self):
        """
        Returns the number of available spots to be subscribed to the delivery station.
        :return: Number of possible available spots.
        """
        count = 0
        if self.is_delivery_spot_free():
            count += 1

        if self.next_waiting_index is not None:
            for index in range(self.next_waiting_index, self.queue_size):
                if tuple(self.waiting_locations[index]) not in self.solver.shelves_locations:
                    count += 1

        return count

    def unsubscribe_task(self, task):
        """
        Unsubscribes a previously assigned task
        :param task: the task to be unsubscribed.
        :return: the (updated) task
        """
        if self.current_processed_task == task:
            # Unsubscribing the head task which is currently assigned to the delivery station
            self.current_processed_task = None
            #self.current_waiting_and_service_time -= self.time_to_finish_current_task
            self.progress_queue(-1)
        else:
            task_found = False

            for i in range(len(self.tasks_queue)):
            # i = 0
            # while i < len(self.tasks_queue):
                if self.tasks_queue[i] == task:
                    task_found = True
                    self.current_waiting_and_service_time -= self.tasks_service_times[i]
                    del self.tasks_queue[i]
                    del self.tasks_service_times[i]
                    self.progress_queue(i)
                    break

            # Asserting that the task is in the queue
            if not task_found:
                print('Warning! Exception in DeliveryStation class: unsubscribe_task() was called with an unrecognized task')
                # exit(1)
        '''
        if self.delivery_pos == (18, 1):
            if (self.next_waiting_index == None and len(self.tasks_queue) != self.queue_size) or\
                (self.next_waiting_index is not None and self.next_waiting_index != len(self.tasks_queue)):
                print('problem')
                exit(1)
        '''

        return task

    def subscribe_task(self, task):
        """
        Subscribes a new task to the delivery station.
        :param task: a new task to be added.
        :return: the (updated) task
        """
        # Asserting that there's available spot
        if not self.is_available():
            print('Exception in DeliveryStation ', self.delivery_pos, ' : subscribe_task() was called without any free spots left')
            exit(1)

        if self.is_random_service_time:
            task_service_time = np.random.randint(1, 10, 1)
        else:
            task_service_time = self.service_time

        # Checks whether the delivery spot is free and no waiting is required
        if self.is_delivery_spot_free():
            # The task can be pointed directly to the delivery spot itself
            self.current_processed_task = task

            self.time_to_finish_current_task = task_service_time
            self.current_waiting_and_service_time = self.time_to_finish_current_task
            task.current_destination = self.delivery_pos
        else:
            # The delivery spot is occupied, thus the task is queued
            task.current_destination = deepcopy(self.waiting_locations[self.next_waiting_index])
            self.tasks_queue.append(task)
            self.tasks_service_times.append(task_service_time)
            self.current_waiting_and_service_time += task_service_time

            # Asserting that queue is not full yet
            self.next_waiting_index += 1
            if self.next_waiting_index == self.queue_size:
                # The queue is full
                self.next_waiting_index = None
        '''
        if self.delivery_pos == (18, 1):
            if (self.next_waiting_index == None and len(self.tasks_queue) != self.queue_size) or \
                    (self.next_waiting_index is not None and self.next_waiting_index != len(self.tasks_queue)):
                print('problem')
                exit(1)
        '''
        return task

    def progress_queue(self, index):
        """
        Progresses the waiting tasks in the queue given that a task in position 'index'  is leaving the queue.
        :param index: the queue index of a task that has finished/unsubscribed:
               Index == -1 : If the currently processed task is finished/unsubscribed.
               Index >= 0  : A waiting task in the queue has been unsubscribed.
        :return: None
        """
        # Asserting that there are waiting tasks in the queue, exiting if not
        if len(self.tasks_queue) == 0:
            index = 0

        some_agent_moved = False

        if index == -1:
            self.current_waiting_and_service_time -= self.time_to_finish_current_task

            # Signal the first task waiting in queue to advance towards the delivery post
            next_task = self.tasks_queue.pop(0)
            self.current_processed_task = next_task
            agent_name = self.solver.tasks_to_agents[next_task.task_name]
            agent_record = self.solver.agents_dict[agent_name]
            # agent_record['goal'][0] = self.delivery_pos[0]
            # agent_record['goal'][1] = self.delivery_pos[1]
            agent_record['goal'] = deepcopy(self.delivery_pos)
            next_task.current_destination = deepcopy(self.delivery_pos)
            self.time_to_finish_current_task = self.tasks_service_times.pop(0)
            some_agent_moved = True
            queue_progress_start_ind = 0

        else:
            queue_progress_start_ind = index

        # Reseting the next waiting index if necessary
        # if len(self.tasks_queue) == 0:
        #     self.next_waiting_index = 0

        # Advance any waiting tasks in the queue (if there are any)
        for i in range(queue_progress_start_ind, len(self.tasks_queue)):
            next_task = self.tasks_queue[i]
            agent_name = self.solver.tasks_to_agents[next_task.task_name]
            agent_record = self.solver.agents_dict[agent_name]
            waiting_location = deepcopy(self.waiting_locations[i])
            # Asserting that the waiting location is actually free
            if tuple(waiting_location) not in self.solver.shelves_locations:
                agent_record['goal'] = waiting_location
                next_task.current_destination = waiting_location
                some_agent_moved = True
            else:
                break  # The queue is stuck, no need to advance the next agents waiting in line

            '''
            # If last waiting task has advanced, need to update the index of the next available waiting spot
            if i == len(self.tasks_queue) - 1:
                if self.next_waiting_index is None:
                    self.next_waiting_index = len(self.waiting_locations) - 1
                else:
                    self.next_waiting_index -= 1
            '''
        # If last waiting task has advanced, need to update the index of the next available waiting spot
        if len(self.tasks_queue) < self.queue_size and some_agent_moved:
            if self.next_waiting_index is None:
                self.next_waiting_index = len(self.waiting_locations) - 1
            else:
                self.next_waiting_index = max(0, self.next_waiting_index - 1)

        if index == len(self.tasks_queue):
            self.next_waiting_index = index

    def time_step(self):
        """
        Performs a single time step of the delivery station:
        1. Decrements the expected service time of the current processed task
        2. If the currently processed task has finished its service, setting its destination back to the pick-up
        3. Advancing the waiting tasks in the queue (if there are any)
        :return: None
        """
        if (self.next_waiting_index == None and len(self.tasks_queue) != self.queue_size) or \
                (self.next_waiting_index is not None and self.next_waiting_index != len(self.tasks_queue)):
            print('problem at delivery station ',self.delivery_pos)
            exit(1)



        # Checking if there's a currently processed task in the delivery spot
        if self.current_processed_task is not None:
            # A processed task is in process
            agent_name = self.solver.tasks_to_agents[self.current_processed_task.task_name]
            agent_record = self.solver.agents_dict[agent_name]

            # Asserting that agent has actually arrived at the delivery position
            if agent_record['current_pos'] == self.delivery_pos:
                self.time_to_finish_current_task -= 1
                self.current_waiting_and_service_time -= 1

                # Check if the  current task has finished its processing
                if self.time_to_finish_current_task == 0:
                    self.solver.tasks[self.current_processed_task.task_name].task_state = TaskState.DELIVERY2PICKUP
                    agent_record['goal'] = self.solver.tasks[self.current_processed_task.task_name].pickup_pos
                    self.current_processed_task = None

                    if len(self.tasks_queue) > 0 and self.is_delivery_spot_free():
                        self.progress_queue(-1)

                    '''        
                    # Signal the first task waiting in queue to advance towards the delivery post
                    next_task = self.tasks_queue.pop()
                    agent_name = self.solver.tasks_to_agents(next_task.task_name)
                    agent_record = self.solver.agents_dict[agent_name]
                    agent_record['goal'][0] = self.delivery_pos[0]
                    agent_record['goal'][1] = self.delivery_pos[1]

                    # Advance any waiting tasks in the queue (if there are any)
                    for i in range(len(self.tasks_queue)):
                        next_task = self.tasks_queue[i]
                        agent_name = self.solver.tasks_to_agents(next_task.task_name)
                        agent_record = self.solver.agents_dict[agent_name]
                        waiting_location = self.waiting_locations[i - 1]
                        # Asserting that the waiting location is actually free
                        if waiting_location not in self.solver.shelves_locations:
                            agent_record['goal'][0] = self.waiting_locations[i - 1][0]
                            agent_record['goal'][1] = self.waiting_locations[i - 1][1]
                        else:
                            break  # The queue is stuck, no need to advance the next agents waiting in line
                    '''

        else:
            # Check if there are waiting tasks to be progressed
            if len(self.tasks_queue) > 0 and self.is_delivery_spot_free():
                self.progress_queue(0)

                '''
                # Signal the first task waiting in queue to advance towards the delivery post
                next_task = self.tasks_queue.pop()
                agent_name = self.solver.tasks_to_agents(next_task.task_name)
                agent_record = self.solver.agents_dict[agent_name]
                agent_record['goal'][0] = self.delivery_pos[0]
                agent_record['goal'][1] = self.delivery_pos[1]

                # Advance any waiting tasks in the queue (if there are any)
                for i in range(len(self.tasks_queue)):
                    next_task = self.tasks_queue[i]
                    agent_name = self.solver.tasks_to_agents(next_task.task_name)
                    agent_record = self.solver.agents_dict[agent_name]
                    agent_record['goal'][0] = self.waiting_locations[i - 1][0]
                    agent_record['goal'][1] = self.waiting_locations[i - 1][1]
                '''