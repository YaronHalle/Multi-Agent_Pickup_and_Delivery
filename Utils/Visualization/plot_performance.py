import matplotlib
import matplotlib.pyplot as plt
import json
import argparse
import numpy as np
from Simulations.classes import *
def plot_tasks_per_states(data):
    time_ar = [int(str) for str in list(data.keys())]
    pending_ar = [record['tasks_counters']['0'] for record in data.values()]
    assigned_ar = [record['tasks_counters']['1'] for record in data.values()]
    executed_ar = [record['tasks_counters']['2'] for record in data.values()]

    plt.plot(time_ar, pending_ar, color='y', label='Pending')
    plt.plot(time_ar, assigned_ar, color='g', label='Assigned')
    plt.plot(time_ar, executed_ar, color='r', label='Executed')

    plt.xlabel('Iteration number')
    plt.ylabel('Tasks count')
    plt.title('Tasks state distribution')

    plt.legend()
    plt.grid()
    plt.show()

def plot_perf_metrics(data):
    time_ar = [int(str) for str in list(data.keys())]
    throughput_ar = [record['avg_throughput'] for record in data.values()]
    servicetime_ar = [record['avg_service_time'] for record in data.values()]
    delaytime_ar = [record['avg_delay_time'] for record in data.values()]

    fig_title = 'Steady state throughput = ' + '{:.2f}'.format(throughput_ar[-1]) + '\n' + \
            'Steady state service time = ' + '{:.2f}'.format(servicetime_ar[-1]) + '\n' + \
            'Steady state delay time = ' + '{:.2f}'.format(delaytime_ar[-1])

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    fig.suptitle('Performance vs Time', fontweight = 'bold')

    ax1.plot(time_ar, throughput_ar, label='Average Throughput\n' + '[Last value={:.2f}'.format(throughput_ar[-1]) + ']')
    ax1.grid()
    ax1.legend()
    #ax1.title(fig_title)

    ax2.plot(time_ar, servicetime_ar, color='g', label='Average Service Time\n' + '[Last value={:.2f}'.format(servicetime_ar[-1]) + ']')
    ax2.plot(time_ar, delaytime_ar, color='r', label='Average Delay Time\n' + '[Last value={:.2f}'.format(delaytime_ar[-1]) + ']')
    ax2.grid()
    ax2.legend()

    plt.xlabel('Iteration number')

    plt.show()

def compare_atomic_vs_non_atomics(atomic_data, non_atomic_data):
    time_ar = [int(str) for str in list(atomic_data.keys())]
    throughput_atomic_ar = [record['avg_throughput'] for record in atomic_data.values()]
    servicetime_atomic_ar = [record['avg_service_time'] for record in atomic_data.values()]
    delaytime_atomic_ar = [record['avg_delay_time'] for record in atomic_data.values()]

    throughput_non_atomic_ar = [record['avg_throughput'] for record in non_atomic_data.values()]
    servicetime_non_atomic_ar = [record['avg_service_time'] for record in non_atomic_data.values()]
    delaytime_non_atomic_ar = [record['avg_delay_time'] for record in non_atomic_data.values()]

    fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    fig.suptitle('Atomic vs Non-Atomic performance comparison', fontweight='bold')

    # Average throughput
    ax1.plot(time_ar, throughput_atomic_ar, color='b',
             label='Atomic\n' + '[Last value={:.2f}'.format(throughput_atomic_ar[-1]) + ']')
    ax1.plot(time_ar, throughput_non_atomic_ar, color='r',
             label='Non-Atomic\n' + '[Last value={:.2f}'.format(throughput_non_atomic_ar[-1]) + ']')
    ax1.grid()
    ax1.legend()
    ax1.set_ylabel('Average Throughput')

    # Average service time
    ax2.plot(time_ar, servicetime_atomic_ar, color='b',
             label='Atomic\n' + '[Last value={:.2f}'.format(servicetime_atomic_ar[-1]) + ']')
    ax2.plot(time_ar, servicetime_non_atomic_ar, color='r',
             label='Non-Atomic\n' + '[Last value={:.2f}'.format(servicetime_non_atomic_ar[-1]) + ']')
    ax2.grid()
    ax2.legend()
    ax2.set_ylabel('Average Service Time')

    # Average delay time
    ax3.plot(time_ar, delaytime_atomic_ar, color='b',
             label='Atomic\n' + '[Last value={:.2f}'.format(delaytime_atomic_ar[-1]) + ']')
    ax3.plot(time_ar, delaytime_non_atomic_ar, color='r',
             label='Non-Atomic\n' + '[Last value={:.2f}'.format(delaytime_non_atomic_ar[-1]) + ']')
    ax3.grid()
    ax3.legend()
    ax3.set_xlabel('Iteration number')
    ax3.set_ylabel('Average Delay Time')

    plt.show()

def plot_non_atomic_performance(non_atomic_data):
    time_ar = [int(str) for str in list(non_atomic_data.keys())]
    agents_involved_ar = [record['non_atomic_perf'][0] for record in non_atomic_data.values()]
    improvement_ar = [record['non_atomic_perf'][1] for record in non_atomic_data.values()]

    soc_improvement = np.array([])
    number_of_agents = np.array([])

    splitting_counter = 0
    for record in non_atomic_data.values():
        if record['non_atomic_perf'][0] > 0:
            splitting_counter += 1
            number_of_agents = np.append(number_of_agents, record['non_atomic_perf'][0])
            soc_improvement = np.append(soc_improvement, record['non_atomic_perf'][1])

    portion_of_splittings = splitting_counter / len(time_ar) * 100

    print(f"% iterations of task splitting = {portion_of_splittings}")
    print(f"Avg. % SoC improvement = {soc_improvement.mean()}")
    print(f"Avg. number of involved agents = {number_of_agents.mean()}")

    # fig_title = 'Steady state throughput = ' + '{:.2f}'.format(throughput_ar[-1]) + '\n' + \
    #             'Steady state service time = ' + '{:.2f}'.format(servicetime_ar[-1]) + '\n' + \
    #             'Steady state delay time = ' + '{:.2f}'.format(delaytime_ar[-1])

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    fig.suptitle(f'Non-Atomic MAPD performance\n(Splitting frequency : {portion_of_splittings}%)', fontweight='bold')
    plt.axes(ax1)
    plt.bar(time_ar, agents_involved_ar, width=1.5)
    # ax1.grid()
    ax1.set_ylabel('Number of agents \nthat dropped shelves')
    # ax1.legend()
    # ax1.title(fig_title)

    plt.axes(ax2)
    plt.bar(time_ar, improvement_ar, color='r', width=1.5)
    # ax2.grid()
    ax2.set_ylabel('% improvement in SoC')
    # ax2.legend()

    plt.xlabel('Iteration number')

    plt.show()

def get_average_non_atomic_results(non_atomic_data):
    pass

if __name__ == "__main__":
    '''
    combined_stats_file = b"D:/GitHub/Multi-Agent_Pickup_and_Delivery/Utils/Visualization/combined_5_runs_stats_file.json"
    with open(combined_stats_file, "r") as read_file:
        combined_data = json.load(read_file)
    atomic_data = combined_data['atomic']
    non_atomic_data = combined_data['non_atomic']
    '''

    '''
    atomic_file = b"D:/Results/1000steps_small_warehouse_50iterations_lns/1/atomic/stats_file.json"
    with open(atomic_file, "r") as read_file:
        atomic_data = json.load(read_file)
         '''

    non_atomic_file = b"D:/Results/1000steps_small_warehouse_50iterations_lns/5/non_atomic/stats_file.json"
    with open(non_atomic_file, "r") as read_file:
        non_atomic_data = json.load(read_file)


    # compare_atomic_vs_non_atomics(atomic_data, non_atomic_data)

    plot_non_atomic_performance(non_atomic_data)

    # del data['tasks_counters']
    # del data['current_step_throughput']
    # del data['avg_throughput']
    # del data['avg_service_time']
    # del data['avg_delay_time']
    #
    # with open("stats_file.json", "w") as write_file:
    #     json.dump(data, write_file)

