#!/usr/bin/python
import math
import sys
import re
import numpy as np

import matplotlib
import pylab as plot
import datetime as datetime
import os
import argparse
import pprint

import slamlog
import plotutils
import utils

#########################################################################################
# rendering setting
#########################################################################################
# "ReF",
# "EF",
default_order = [
    "ORB2",
    "ORB3",
    "ReF",
    "EF",
    "FULL",
    "OVINS"
]


#########################################################################################
# UTILS PLOT
#########################################################################################
def generate_latex_doc(containt):
    latex_str = ""
    latex_str += "\\documentclass[landscape]{article}\n"
    latex_str += "\\usepackage[table]{xcolor}\n"
    latex_str += "\\usepackage[ margin=0cm]{geometry}\n"
    latex_str += "\\setlength{\\arrayrulewidth}{0.2mm}\n"
    latex_str += "\\setlength{\\tabcolsep}{2pt}\n"
    latex_str += "\\renewcommand{\\arraystretch}{1}\n"
    latex_str += "\\begin{document}\n"

    latex_str += containt

    latex_str += "\\end{document}\n"

    return latex_str


def generate_latex_table(data,
                         table_title,
                         value_name,
                         value_type,
                         precision="%.2f",
                         high_threshold=None,
                         low_threshold=None):
    dataset_labels = []

    for a in data:
        dataset_labels = list(set(dataset_labels + data[a].keys()))

    algorithm_labels = data.keys()

    latex_str = ""
    latex_str += table_title + "\\\\\n"
    latex_str += "{\\rowcolors{3}{black!10}{black!2}\n"
    latex_str += "\\tiny\n"
    latex_str += "\\begin{tabular}{%s}\n" % ("|l|" + "|".join(["l" for x in dataset_labels]) + "|")
    latex_str += "\\hline\n"
    #    latex_str +=  "Dataset name & \\multicolumn{4}{c|}{ICLNUIM Dataset} & \\multicolumn{19}{c|}{TUM Dataset}  & \\multicolumn{11}{c|}{EuRoC MAV Dataset}  \\\\\n"
    #    latex_str +=  "\\hline\n"
    #    latex_str +=  "Scene name & \\multicolumn{4}{c|}{Living Room} & \\multicolumn{7}{c|}{Freiburg 1} & \\multicolumn{12}{c|}{Freiburg 2} & \\multicolumn{5}{c|}{Machine Hall} & \\multicolumn{3}{c|}{Vicon Room 1} & \\multicolumn{3}{c|}{Vicon Room 2}\\\\\n"
    latex_str += "Algorithm & " + " & ".join([x.replace("_", "\_") for x in dataset_labels]) + "\\\\\n"
    latex_str += "\\hline\n"
    for algorithm in data:
        latex_str += algorithm.replace("_", "\_")
        for dataset in dataset_labels:
            accuracy = None

            if not dataset in data[algorithm].keys():
                latex_str += "& - "
                continue

            if len(data[algorithm][dataset]) == 0:
                utils.printerr("Cannot find any iteration in %s data.\n" % (dataset, algorithm))
                exit(1)

            for it in data[algorithm][dataset]:

                if not plotutils.STATISTICS_SECTION in it.keys():
                    utils.printerr("Invalid data (%s,%s), no STATISTICS_SECTION.\n" % (algorithm, dataset))
                    exit(1)

                if not plotutils.ATE_COLUMN in it[plotutils.STATISTICS_SECTION].keys():
                    utils.printerr("Invalid data(%s,%s), no ATE_COLUMN.\n" % (algorithm, dataset))
                    exit(1)

            for it in data[algorithm][dataset]:
                if accuracy == None:
                    accuracy = 0.0
                accuracy += float(it[plotutils.STATISTICS_SECTION][value_name][value_type])
            accuracy = accuracy / len(data[algorithm][dataset])

            if accuracy == None:
                latex_str += "& - "
            elif high_threshold and accuracy > high_threshold:
                latex_str += "& $>$" + precision % high_threshold
            elif low_threshold and accuracy < low_threshold:
                latex_str += " & \\textbf{" + precision % accuracy + "} "
            else:
                latex_str += " & " + precision % accuracy
        latex_str += "\\\\\n"
    latex_str += "\\hline\n"
    latex_str += "\\end{tabular}\n"
    latex_str += "\n"

    return latex_str


def generate_latex(data):
    tables = ""
    tables += generate_latex_table(data, "ATE Mean", plotutils.ATE_COLUMN, plotutils.MEAN_FIELD, precision="%.2f",
                                   high_threshold=1, low_threshold=0.1)
    tables += generate_latex_table(data, "ATE Max", plotutils.ATE_COLUMN, plotutils.MAX_FIELD, precision="%.2f",
                                   high_threshold=1, low_threshold=0.1)
    tables += generate_latex_table(data, "Frame count", plotutils.ATE_COLUMN, plotutils.COUNT_FIELD, precision="%d")
    return generate_latex_doc(tables)


#########################################################################################
#  MAIN
#########################################################################################
def main():
    # GET ARGUMENTS
    parser = argparse.ArgumentParser()
    parser.add_argument('dir', type=str, help='Files to process')
    parser.add_argument('--latex', action="store_true", help='Generate the latex file')
    parser.add_argument('--violins', action="store_true", help='Generate the violins data')
    parser.add_argument('--plot', type=str, help='Generate the violins pdf file')
    parser.add_argument('--recurse', type=bool, help='Go through directories recursively')
    parser.add_argument('--multiplatform', type=bool, help='Go through directories recursively')

    args = parser.parse_args()
    one_run = {}
    all_means = {}
    if args.multiplatform:
        data = slamlog.load_data_multiplatform(args.dir)
        for platform, value in data.items():
            print(platform)
            all_means[platform] = {}
            for seq, data in sorted(value.items()):
                print(platform, "/", seq)
                violins = plotutils.generate_violins(data)
                output_path = os.path.join(args.dir, platform)
                output_path = os.path.join(output_path, seq)
                print(output_path)
                if seq in one_run:
                    one_run[seq][platform] = violins
                else:
                    one_run[seq] = {platform: violins}
                # plotutils.plot_violins(violins, output_path, default_order)

    for seq in one_run:
        all_means = plotutils.plot_violins_platform(one_run[seq], os.path.join(args.dir, seq), default_order, all_means)
    print(all_means)
    plotutils.plot_means(all_means, os.path.join(args.dir, "means"), default_order)
    plotutils.plot_bars(all_means, os.path.join(args.dir, "bars"), default_order)
    exit(0)
    if args.recurse:
        for path, directories, files in os.walk(args.dir):
            for directory in directories:
                full_path = os.path.join(path, directory)
                if not utils.is_bottom_dir(full_path):
                    continue
                else:
                    print(full_path)
                data = slamlog.load_data_from_input_dirs([full_path])
                if args.latex:
                    latex_str = generate_latex(data)
                    print(latex_str)
                if args.violins:
                    violins = plotutils.generate_violins(data)
                    print(violins)
                if args.plot:
                    if args.multiplatform:
                        violins = plotutils.generate_violins_multiplatform(data)
                        plotutils.plot_violins_multiplatform(violins, os.path.join(full_path, args.plot), default_order)
                    else:
                        violins = plotutils.generate_violins(data)
                        plotutils.plot_violins(violins, os.path.join(full_path, args.plot), default_order)


main()
