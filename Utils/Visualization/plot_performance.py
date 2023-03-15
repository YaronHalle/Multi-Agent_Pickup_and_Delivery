import matplotlib
import matplotlib.pyplot as plt
import json
import argparse
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-stats_file", help="stats_file.json full path")
    args = parser.parse_args()

    with open(args.stats_file, "r") as read_file:
        data = json.load(read_file)

    plot_tasks_per_states(data)
    plot_perf_metrics(data)

    # del data['tasks_counters']
    # del data['current_step_throughput']
    # del data['avg_throughput']
    # del data['avg_service_time']
    # del data['avg_delay_time']
    #
    # with open("stats_file.json", "w") as write_file:
    #     json.dump(data, write_file)

