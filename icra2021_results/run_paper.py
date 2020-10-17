import json
import os
import sys
import subprocess
import signal


def segfault_handler(signum, frame):
    print("segfault")


def abort_handler(signum, frame):
    print("abort")


signal.signal(signal.SIGABRT, abort_handler)
signal.signal(signal.SIGSEGV, segfault_handler)


path = os.getcwd()

with open(sys.argv[1]) as f:
    config = json.load(f)

run_config_array = []
log_filenames_array = []

DATASETS_PATH = path + "/datasets/"
LIBS_PATH = path + "/build/lib/"
SLAMBENCH_EXECUTABLE = path + "/build/bin/slambench"
CUDA_SUFFIX = "-cuda-library.so"
ORIGINAL_SUFFIX = "-original-library.so"

seq_dir = path + "/icra2021_results/results/seq/"
lifelong_dir = path + "/icra2021_results/results/lifelong/"
algorithms_run_dir = path + "/icra2021_results/results/algorithms/"
os.makedirs(seq_dir, exist_ok=True)
os.makedirs(lifelong_dir, exist_ok=True)
os.makedirs(algorithms_run_dir, exist_ok=True)

# Check if suffix is original or cuda
for dataset, values in config.items():
    dataset_seq = seq_dir + dataset + "/"
    os.makedirs(dataset_seq, exist_ok=True)
    for sequence, runs in values.items():
        dataset_fullpath = DATASETS_PATH + dataset + "/" + sequence
        if not os.path.isfile(dataset_fullpath):
            print("Dataset not found, please make sure all datasets are built. Skipping,", dataset_fullpath)
            continue
        sequence_name = os.path.splitext(os.path.basename(sequence))[0] # get sequence name without extension
        sequence_dir = dataset_seq + sequence_name
        os.makedirs(sequence_dir, exist_ok=True)

        for libname, arguments in runs.items():
            if os.path.isfile(LIBS_PATH + "lib" + libname + CUDA_SUFFIX):
                lib_filename = LIBS_PATH + "lib" + libname + CUDA_SUFFIX
            elif os.path.isfile(LIBS_PATH + "lib" + libname + ORIGINAL_SUFFIX):
                lib_filename = LIBS_PATH + "lib" + libname + ORIGINAL_SUFFIX
            else:
                print("UNKNOWN OR INVALID LIBRARY PATH!!!", libname)
            run_config = [SLAMBENCH_EXECUTABLE, "-a", "umeyama" "-i", dataset_fullpath, "-load", lib_filename]
            run_config.extend(arguments)
            log_file = sequence_dir + "/" + libname
            run_config_array.append(run_config)
            log_filenames_array.append(log_file)

runs = 1

if len(sys.argv) > 2:
    runs = int(sys.argv[2])
    print("RUNS:", runs)
for config_no in range(0, len(run_config_array)):
    for run in range(0, runs):
        log_pathname = log_filenames_array[config_no] + "_run" + str(run) + ".log"
        output_args = ["-o", log_pathname]
        final_config = run_config_array[config_no] + output_args
        try:
            subprocess.run(final_config, timeout=1800)
        except subprocess.TimeoutExpired:
            print(final_config, "Timed out!")
