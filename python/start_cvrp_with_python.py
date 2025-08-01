import os
import subprocess
import multiprocessing
import time

env = os.environ.copy()

# Define the directory path and command components
directory_path = os.getcwd()
command_base =  os.path.join(directory_path,"build/Release/bin/Release/3L-VehicleRoutingApplication.exe")
output_folder = r"H:\Data\Results_Heuristic\Heuristic_Classifier_AdaptedUsage/"
input_folder = os.path.join(directory_path,"data/input/3l-cvrp/gendreau/")
parameter_files = [os.path.join(directory_path,"data/input/3l-cvrp/parameters/BenchmarkParameters_AllConstraints.json"),
                   os.path.join(directory_path,"data/input/3l-cvrp/parameters/BenchmarkParameters_AllConstraints_Run2.json"),
                   os.path.join(directory_path,"data/input/3l-cvrp/parameters/BenchmarkParameters_AllConstraints_Run3.json")]
number_of_processes = 6

# Navigate to the directorys

def add_tasks(task_queue, folder_path=input_folder):
    # List all files in the folder
    file_list = os.listdir(folder_path)
    for file_name in file_list:
        full_path = os.path.join(folder_path, file_name)
        if not os.path.isfile(full_path):
            continue
        for parameter_file in parameter_files:

            for seed_offset in range(3):  # seeds 0, 1, 2
                task_queue.put((file_name, parameter_file, seed_offset))

    return task_queue

def run_Heuristic_exe(filename, parameter_file, seed_offset, counter, lock):

    command = [
        command_base,
        "-i", input_folder,
        "-f", filename,
        "-o", output_folder,
        "-p", parameter_file,
        "-s", str(seed_offset)
    ]

    try:
        print(f"Starting: {filename} with seed {seed_offset}\n")
        print(f"Command: {command}\n")

        subprocess.run(command, check=True, capture_output=True, cwd=directory_path, env=env)

        print(f"Finished: {filename} with seed {seed_offset}\n")

        with lock:
            counter.value += 1

    except subprocess.CalledProcessError as e:
        print(f"Failed: {filename} with seed {seed_offset}")
        print(e)
        print(f"Return code: {e.returncode}")
        print("STDERR:", e.stderr.decode())

def process_tasks(task_queue, counter, lock):
    while not task_queue.empty():
        try:
            filename, parameter_file, seed_offset = task_queue.get_nowait()
        except:
            break  # In case of race condition
        run_Heuristic_exe(filename, parameter_file, seed_offset, counter, lock)
    return True

def run():
    empty_task_queue = multiprocessing.Queue()
    full_task_queue = add_tasks(empty_task_queue)

    #Create shared counter and lock
    counter = multiprocessing.Value('i', 0)
    lock = multiprocessing.Lock()
    start_time = time.time()

    processes = []
    for _ in range(number_of_processes):
        p = multiprocessing.Process(target=process_tasks, args=(full_task_queue,counter, lock))
        processes.append(p)
        p.start()

        time.sleep(1)

    for p in processes:
        p.join()

    end_time = time.time()
    elapsed = end_time - start_time
    print(f"\nTotal subprocesses executed: {counter.value} in {elapsed} seconds")

if __name__ == "__main__":
    run()

