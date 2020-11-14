#!/usr/bin/python2

import os
import time
import re
import math
import numpy as np

import utils
import numpy as np
import plotutils
LIBRARY_NAME_PROPERTY = "load-slam-library"

PROPERTIES_SECTION = "Properties"
STATISTICS_SECTION = "Statistics"

FRAME_NUMBER_COLUMN = "Frame Number"

JETSON = "Jetson AGX"
WORKSTATION = "Workstation"
Laptop = "Laptop"
ATE_COLUMN = "AbsoluteError"
CPU_MEMORY_COLUMN = "CPU_Memory"
DURATION_COLUMN = "Duration_Frame"

FPS_COLUMN = "FPS"

MAX_FIELD = "MAX"
MIN_FIELD = "MIN"
MEAN_FIELD = "MEAN"
COUNT_FIELD = "COUNT"
MEDIAN_FIELD = "MEDIAN"
VALID_NUMS = "VALID_NUM"

MAX_SUFFIX = "_MAX"
MIN_SUFFIX = "_MIN"
MEAN_SUFFIX = "_MEAN"
COUNT_SUFFIX = "_COUNT"
MEDIAN_SUFFIX = "_MEDIAN"

######## PARSERS ########
def load_data_from_input_dirs(input_dirs):
    filelist = []
    for dirname in input_dirs:
        try:
            filelist += [os.path.join(dirname, f) for f in os.listdir(dirname) if
                         f[-4:] == ".log" and os.path.isfile(os.path.join(dirname, f))]
        except OSError:
            utils.printerr("Working directory %s not found.\n" % dirname)
            return None
    utils.printinfo("%d files to load ...\n" % len(filelist))
    data = load_data_from_files(filelist)
    return data


def compute_seq_mean(algo_runs):
    algo_means = []
    for dataset in algo_runs:
        for it in algo_runs[dataset]:
            mean = it[STATISTICS_SECTION][ATE_COLUMN][MEAN_FIELD]
            algo_means.append(mean)
    return np.mean(algo_means)


######## PARSERS ########
def load_data_multiplatform(input_dir):
    data = {}
    order = [
        "ORB2",
        "ORB3",
        "OVINS"
    ]
    for platform in os.listdir(input_dir):
        platform_dir = os.path.join(input_dir, platform)
        platform_data = {}
        platform_means = {}
        if not os.path.isdir(platform_dir):
            continue
        platform_mean = {}
        for seq in os.listdir(platform_dir):
            seq_dir = os.path.join(platform_dir, seq)
            if not os.path.isdir(seq_dir):
                continue

            filelist = []
            try:
                filelist += [os.path.join(seq_dir, f) for f in os.listdir(seq_dir) if
                             f[-4:] == ".log" and os.path.isfile(os.path.join(seq_dir, f))]
            except OSError:
                utils.printerr("Working directory %s not found.\n" % seq_dir)
                return None
            utils.printinfo("%d files to load ...\n" % len(filelist))
            seq_data = load_data_from_files(filelist)
            platform_data[seq] = seq_data
            for algo in seq_data:
                seq_mean = compute_seq_mean(seq_data[algo])
                if algo in platform_mean:
                    platform_mean[algo].append(seq_mean)
                else:
                    platform_mean[algo] = [seq_mean]

            # ATE_DATA = platform_data[algo][ATE_COLUMN]
        platform_means[platform] = platform_mean

        data[platform] = platform_data
    # plotutils.generate_violins_multiplatform(data)
    # plotutils.plot_violins_platform(data, "", order)
    return data


def load_data_from_file(filename):
    start_time = time.time()

    f = open(filename)
    raw = f.read()
    f.close()

    inside = None
    headers = None

    data = {"date": None, STATISTICS_SECTION: {}, PROPERTIES_SECTION: {}}
    lines = raw.split("\n")
    load_time = time.time()

    for line in lines:

        if line == "Process every frame mode enabled":
            continue
        if line[0:len("SLAMBench Report run started:")] == "SLAMBench Report run started:":
            matching_header = re.match("SLAMBench Report run started:\s+(.*)", line)
            assert (matching_header)
            data["date"] = str(matching_header.group(1))
            continue

        if re.match(PROPERTIES_SECTION + ":", line):
            inside = PROPERTIES_SECTION
            continue

        if re.match(STATISTICS_SECTION + ":", line):
            inside = STATISTICS_SECTION
            continue

        if line == "":
            continue

        if re.match("=+", line):
            if inside != None:
                continue
            else:
                utils.printerr("Error unknow section.\n")
                return None

        if inside == PROPERTIES_SECTION:
            matching_arguments = re.match("\s*(.*):\s+(.*)\s*", line)
            if matching_arguments:
                data[PROPERTIES_SECTION][matching_arguments.group(1)] = matching_arguments.group(2)
                continue

        if inside == STATISTICS_SECTION:
            matching_fields = line.split("\t")
            if matching_fields:
                if headers and len(headers) == len(matching_fields):
                    for i in range(len(matching_fields)):
                        current_value = float("NaN")
                        try:
                            current_value = float(matching_fields[i])
                        except ValueError:
                            current_value = float("NaN")
                        data[STATISTICS_SECTION][headers[i]] += [current_value]
                        # if math.isnan(float(matching_fields[i])) :
                        #    utils.printerr ( INVALID + " %s : Error while parsing the file, NaN found.\n" % filename )
                        #    return None
                    continue
                else:
                    if headers:
                        utils.printerr("New \n")
                    headers = matching_fields[:]
                    for k in headers:
                        if not k in data[STATISTICS_SECTION].keys():
                            data[STATISTICS_SECTION][k] = []
                    continue

        utils.printerr(
            "[load_data_from_file('%s')] Error line not parsed inside '%s': '%s'\n" % (filename, inside, line))
        return None
    loop_time = time.time()

    if headers == None:
        return None
    # print "load = %f" % (1000 * (load_time - start_time) )
    # print "loop = %f" % (1000 * (loop_time - load_time) )

    return data


def turn_data_to_stats(data):
    stats = {}
    if not data or STATISTICS_SECTION not in data.keys():
        utils.printerr("no data or no STATISTICS_SECTION in data.keys()\n")
        return None
    if FRAME_NUMBER_COLUMN not in data[STATISTICS_SECTION].keys():
        utils.printerr("no '%s' in data[STATISTICS_SECTION].keys()\n" % FRAME_NUMBER_COLUMN)
        utils.printerr("data[STATISTICS_SECTION].keys() = %s\n" % data[STATISTICS_SECTION].keys())
        return None
    frame_count = len(data[STATISTICS_SECTION][FRAME_NUMBER_COLUMN])
    last_algorithm_name = None

    for k in data[STATISTICS_SECTION].keys():
        matching_key = re.match("^((.+)-)?(.+)$", k)

        if not matching_key:
            utils.printerr("Error with '%s' does not match any known field names.\n" % k)
            utils.printerr("Statistics header was :\n%s\n" % data[STATISTICS_SECTION].keys())
            return None
        row_name = matching_key.group(3)
        if row_name not in stats.keys():
            stats[row_name] = {}
        valid_numbers = [x for x in data[STATISTICS_SECTION][k] if not math.isnan(float(x))]
        if len(valid_numbers) > 0:
            stats[row_name] = {
                COUNT_FIELD: len(valid_numbers),
                MIN_FIELD: min(valid_numbers),
                MAX_FIELD: max(valid_numbers),
                MEDIAN_FIELD: np.median(valid_numbers),
                MEAN_FIELD: np.mean(valid_numbers),
                VALID_NUMS: valid_numbers
            }
        else:
            stats[row_name] = {
                COUNT_FIELD: len(valid_numbers),
                MIN_FIELD: float("NaN"),
                MAX_FIELD: float("NaN"),
                MEDIAN_FIELD: float("NaN"),
                MEAN_FIELD: float("NaN"),
            }

    return stats


def load_data_from_files(filelist):
    data = {}
    for filename in filelist:

        start_time = time.time()
        temp = load_data_from_file(filename)
        load_time = (time.time() - start_time) * 1000

        if temp and STATISTICS_SECTION in temp.keys() and PROPERTIES_SECTION in temp.keys():
            stats = turn_data_to_stats(temp)

            if not "input" in temp[PROPERTIES_SECTION].keys():
                utils.printerr(" %s : input argument not found.\n" % filename)
                continue

            dataset = temp[PROPERTIES_SECTION]["input"]
            # dataset = dataset.split("/")[-1]

            libraryname = temp[PROPERTIES_SECTION][LIBRARY_NAME_PROPERTY]

            if stats is None:
                utils.printerr(" %s : stats == None.\n" % filename)
                continue

            if libraryname not in data.keys():
                data[libraryname] = {}

            if dataset not in data[libraryname].keys():
                data[libraryname][dataset] = []

            data[libraryname][dataset] += [{PROPERTIES_SECTION: temp[PROPERTIES_SECTION],
                                            "date": temp["date"],
                                            STATISTICS_SECTION: stats}]
            utils.printinfo(" %s loaded in %f ms : len of stats = %d\n" % (filename, load_time, len(stats.keys())))
        else:
            utils.printerr(" %s : Error while loading the file. \n" % (filename))

    return data


def flat_data(data_array, colsX, colsY):
    main = {}
    for run in data_array:
        wee = {}
        for col in run[PROPERTIES_SECTION]:
            if col in colsX:
                wee[col] = run[PROPERTIES_SECTION][col]

        for col in run[STATISTICS_SECTION]:
            for subcol in run[STATISTICS_SECTION][col].keys():
                if col + "_" + subcol in colsY:
                    wee[col + "_" + subcol] = run[STATISTICS_SECTION][col][subcol]
        if len(main.keys()) != 0 and len(main.keys()) != len(wee.keys()):
            utils.printerr("Invalid datapoint.\n")
            exit(1)

        for x in colsX + colsY:
            if not x in wee.keys():
                utils.printerr("%s not found in the log.\n" % x)
                exit(1)

        for row in wee.keys():
            if not row in main:
                main[row] = []
            main[row] += [wee[row]]

    return main
