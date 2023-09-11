import json
import numpy as np

if __name__ == "__main__":
    atomic_avg_results = {}
    non_atomic_avg_results = {}
    results = {}
    atomic_data = []
    non_atomic_data = []

    atomic_files = [b"D:/Results/1000steps_small_warehouse_50iterations_lns/1/atomic/stats_file.json",
                    b"D:/Results/1000steps_small_warehouse_50iterations_lns/2/atomic/stats_file.json",
                    b"D:/Results/1000steps_small_warehouse_50iterations_lns/3/atomic/stats_file.json",
                    b"D:/Results/1000steps_small_warehouse_50iterations_lns/4/atomic/stats_file.json",
                    b"D:/Results/1000steps_small_warehouse_50iterations_lns/5/atomic/stats_file.json"]

    non_atomic_files = [b"D:/Results/1000steps_small_warehouse_50iterations_lns/1/non_atomic/stats_file.json",
                        b"D:/Results/1000steps_small_warehouse_50iterations_lns/2/non_atomic/stats_file.json",
                        b"D:/Results/1000steps_small_warehouse_50iterations_lns/3/non_atomic/stats_file.json",
                        b"D:/Results/1000steps_small_warehouse_50iterations_lns/4/non_atomic/stats_file.json",
                        b"D:/Results/1000steps_small_warehouse_50iterations_lns/5/non_atomic/stats_file.json"]

    # Loading data to memory
    for i in range(len(atomic_files)):
        with open(atomic_files[i], "r") as read_file:
            atomic_data.append(json.load(read_file))
        with open(non_atomic_files[i], "r") as read_file:
            non_atomic_data.append(json.load(read_file))

    # Computing average statistics
    N_iterations = len(atomic_data[0].keys())
    for iter in range(N_iterations):
        # Atomic data
        atomic_avg_results[iter] = {}
        throuhput = np.array([atomic_data[k][str(iter)]['avg_throughput'] for k in range(len(atomic_data))])
        service_time = np.array([atomic_data[k][str(iter)]['avg_service_time'] for k in range(len(atomic_data))])
        delay_time = np.array([atomic_data[k][str(iter)]['avg_delay_time'] for k in range(len(atomic_data))])
        throuhput[throuhput == None] = np.nan
        service_time[service_time == None] = np.nan
        delay_time[delay_time == None] = np.nan
        atomic_avg_results[iter]['avg_throughput'] = np.nanmean(throuhput)
        atomic_avg_results[iter]['avg_service_time'] = np.nanmean(service_time)
        atomic_avg_results[iter]['avg_delay_time'] = np.nanmean(delay_time)

        # Non-Atomic data
        non_atomic_avg_results[iter] = {}
        throuhput = np.array([non_atomic_data[k][str(iter)]['avg_throughput'] for k in range(len(non_atomic_data))])
        service_time = np.array([non_atomic_data[k][str(iter)]['avg_service_time'] for k in range(len(non_atomic_data))])
        delay_time = np.array([non_atomic_data[k][str(iter)]['avg_delay_time'] for k in range(len(non_atomic_data))])
        throuhput[throuhput == None] = np.nan
        service_time[service_time == None] = np.nan
        delay_time[delay_time == None] = np.nan
        non_atomic_avg_results[iter]['avg_throughput'] = np.nanmean(throuhput)
        non_atomic_avg_results[iter]['avg_service_time'] = np.nanmean(service_time)
        non_atomic_avg_results[iter]['avg_delay_time'] = np.nanmean(delay_time)

    results['atomic'] = atomic_avg_results
    results['non_atomic'] = non_atomic_avg_results

    with open("combined_5_runs_stats_file.json", "w") as write_file:
        json.dump(results, write_file)