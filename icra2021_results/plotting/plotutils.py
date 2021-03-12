#!/usr/bin/python2


# DO not use DISPLAY within matplotlib
# import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
# End of BugFix

import matplotlib
import pylab as plot
import os
import slamlog
import utils
import numpy as np
#########################################################################################
# UTILS PLOT
#########################################################################################
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
params = {'legend.fontsize': 22,
          'xtick.labelsize': 20,
          'ytick.labelsize': 20,
          'figure.titlesize': 20,
          # 'legend.linewidth': 2,          
          #          'font.size': 8,
          }
plot.rcParams.update(params)
# font = {
#        'size'  : 18}
# matplotlib.rc('font', **font)


#########################################################################################
# UTILS ALGO
#########################################################################################
labels = {
    'kfusion-cuda': "KF-CUDA",
    'kfusion-opencl': "KF-OCL",
    'kfusion-openmp': "KF-OMP",
    'kfusion-cpp': "KF-CPP",
    'kfusion-notoon': "KF-NOT",

    'kfusion-octree-openmp': "KO-OMP",
    'kfusion-octree-cpp': "KO-CPP",

    'efusion-cuda': "EF",

    'orbslam2-original': "ORB2",
    'ORB_SLAM3-original': "ORB3",
    'refusion-original': "ReF",
    'fullfusion-original': "FULL",
    'open_vins-original': "OVINS",

    'infinitam-cpp': "IT-CPP",
    'infinitam-openmp': "IT-OMP",
    'infinitam-cuda': "IT-CUDA",

    'lsdslam-cpp': "LSD",
    'lsdslam-original_mp': "LSD-PTH",

    'ptam-original_mp': "PTAM",

    'svo-original': "SVO",
    'okvis-original': "OKVIS",
    'monoslam-sequential': "MO",
}

platforms = ["Jetson", "Laptop", "Workstation"]

def getlabel(name):
    for lab in labels.keys():
        if lab in name:
            return labels[lab]
    utils.printerr("Name '%s' not found in labels '%s'\n" % (name, labels))
    exit(1)


data_shortname = {
    "living_room_traj0_loop.slam": "LR0",
    "living_room_traj1_loop.slam": "LR1",
    "living_room_traj2_loop.slam": "LR2",
    "living_room_traj3_loop.slam": "LR3",
    "rgbd_dataset_freiburg1_rpy.slam": "FR1_rpy",
    "rgbd_dataset_freiburg2_rpy.slam": "FR2_rpy",
    "rgbd_dataset_freiburg1_xyz.slam": "FR1_xyz",
    "rgbd_dataset_freiburg2_xyz.slam": "FR2_xyz",
    "rgbd_dataset_freiburg1_360.slam": "FR1_360",
    "rgbd_dataset_freiburg1_desk.slam": "FR1_desk",
    "rgbd_dataset_freiburg1_desk2.slam": "FR1_desk2",
    "rgbd_dataset_freiburg1_floor.slam": "FR1_floor",
    "rgbd_dataset_freiburg1_room.slam": "FR1_room",
    "rgbd_dataset_freiburg2_desk.slam": "FR2_desk",
}


def generate_violins(data):
    violins = {}
    datasets = []
    for a in data:
        datasets = list(set(datasets + [*data[a].keys()]))

    for algorithm in data:
        for dataset in data[algorithm]:
            for it in data[algorithm][dataset]:
                if not slamlog.STATISTICS_SECTION in it.keys():
                    utils.printerr("Invalid data (%s,%s), no STATISTICS_SECTION.\n" % (algorithm, dataset))
                    exit(1)

                if slamlog.ATE_COLUMN not in it[slamlog.STATISTICS_SECTION].keys():
                    utils.printerr("Invalid data(%s,%s), no ATE_COLUMN.\n" % (algorithm, dataset))
                    exit(1)

    for algorithm in data:
        MeanATE = []
        FPS = []
        Memory = []
        for dataset in data[algorithm]:
            for it in data[algorithm][dataset]:
                MeanATE += [it[slamlog.STATISTICS_SECTION][slamlog.ATE_COLUMN][slamlog.MEAN_FIELD]]
                Memory += [it[slamlog.STATISTICS_SECTION][slamlog.CPU_MEMORY_COLUMN][slamlog.MAX_FIELD] / 1000000]
                FPS += [1 / it[slamlog.STATISTICS_SECTION][slamlog.DURATION_COLUMN][slamlog.MEAN_FIELD]]
                # plt.plot(list(range(it[slamlog.STATISTICS_SECTION][slamlog.ATE_COLUMN][slamlog.COUNT_FIELD])), it[slamlog.STATISTICS_SECTION][slamlog.ATE_COLUMN][slamlog.VALID_NUMS])
                # plt.suptitle(os.path.splitext(os.path.basename(dataset))[0] + "/" + os.path.basename(algorithm))
                # plt.ylabel('Absolute Trajectory Error (m)')
                # plt.xlabel('Frame no')
                # plt.show()

        violins[algorithm] = {slamlog.ATE_COLUMN: MeanATE, slamlog.FPS_COLUMN: FPS, slamlog.CPU_MEMORY_COLUMN: Memory}

    return violins


def generate_violins_multiplatform(all_platform_runs):
    violins = {}
    sequences = []
    algorithms = []
    datasets = []
    for platform, data in all_platform_runs.items():
        for seq in data:
            sequences.append(seq)
            for algo in seq:
                algorithms = list(set(algorithms + [*data[seq].keys()]))
    print(sequences)
    print(algorithms)
    print(datasets)
    for platform, data in all_platform_runs.items():
        algo_dict = {}
        for seq in data:
            for algorithm, runs in data[seq].items():
                platform_dict = {}
                MeanATE = []
                for dataset in runs:
                    for it in data[seq][dataset]:
                        if not slamlog.STATISTICS_SECTION in it.keys():
                            utils.printerr("Invalid data (%s,%s), no STATISTICS_SECTION.\n" % (algorithm, dataset))
                            exit(1)

                        if slamlog.ATE_COLUMN not in it[slamlog.STATISTICS_SECTION].keys():
                            utils.printerr("Invalid data(%s,%s), no ATE_COLUMN.\n" % (algorithm, dataset))
                            exit(1)

                        MeanATE.append(it[slamlog.STATISTICS_SECTION][slamlog.ATE_COLUMN][slamlog.MEAN_FIELD])
                        plt.plot(list(range(it[slamlog.STATISTICS_SECTION][slamlog.ATE_COLUMN][slamlog.COUNT_FIELD])), it[slamlog.STATISTICS_SECTION][slamlog.ATE_COLUMN][slamlog.VALID_NUMS])
                        plt.suptitle(os.path.splitext(os.path.basename(dataset))[0] + "/" + os.path.basename(algorithm))
                        plt.ylabel('Absolute Trajectory Error (m)')
                        plt.xlabel('Frame no')
                        plt.show()

            algo_dict[algorithm]
        violins[algorithm] = {slamlog.ATE_COLUMN: MeanATE}

    return violins


def plot_violins_platform(data, filename, order, means):

    violins = data["Laptop"]

    ## PLOT DATA ##

    algo_long_name = [x for x in violins.keys() if getlabel(x) in order]
    algo_short_name = [getlabel(x) for x in violins.keys() if getlabel(x) in order]

    algo_position = [order.index(x) for x in algo_short_name]

    algo_ordered = [x for _, x in sorted(zip(algo_position, algo_long_name))]

    algos = algo_ordered


    fig, axes = plt.subplots(nrows=1, ncols=len(platforms), figsize=(18, 5))
    i = 0

    max_ate = -1
    for platform in platforms:
        if platform not in data:
            utils.printerr("DATA FOR PLATFORM MISSING!!")
        ATE_DATA = []
        LABELS = []
        for algo in algos:
            if algo in data[platform]:
                algo_data = data[platform][algo][slamlog.ATE_COLUMN]
                if algo in means[platform]:
                    means[platform][algo].append(np.mean(algo_data))
                else:
                    means[platform][algo] = [np.mean(algo_data)]
                if np.max(algo_data) > max_ate:
                    max_ate = np.max(algo_data)
                ATE_DATA.append(algo_data)
                LABELS.append(getlabel(algo))
            else:
                utils.printerr(algo + " not available on " + platform + "!\n")
                # ATE_DATA.append([0])
        pos = [x for x in range(len(LABELS))]

        if len(pos) == 0:
            utils.printerr("Empty data, cannot draw anything.\n")
            return False

        parts = axes[i].violinplot(ATE_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True,
                               bw_method='silverman')
        parts['cmeans'].set_linewidth(2)
        parts['cmeans'].set_edgecolor("black")
        axes[i].set_title(platforms[i] + " ATE", fontsize=20)
        axes[i].set_xticks(pos)
        axes[i].set_xticklabels(LABELS)

        for label in axes[i].get_xmajorticklabels():
            label.set_rotation(30)
            label.set_horizontalalignment("right")

        for pc in parts['bodies']:
            if platforms[i] == "Jetson":
                pc.set_facecolor('#D43F3A')
            elif platforms[i] == "Workstation":
                pc.set_facecolor('#008080')
            pc.set_edgecolor('black')
            pc.set_alpha(0.8)

        i += 1
    for ax in axes:
        ax.set_ylim([0, max_ate + max_ate / 10])

    if filename:
        fig.subplots_adjust(hspace=0.4)
        plt.savefig(filename, bbox_inches='tight')
    else:
        plt.show()

    return means


def plot_means(data, filename, order):
    violins = data["Laptop"]

    ## PLOT DATA ##

    algo_long_name = [x for x in violins.keys() if getlabel(x) in order]
    algo_short_name = [getlabel(x) for x in violins.keys() if getlabel(x) in order]

    algo_position = [order.index(x) for x in algo_short_name]

    algo_ordered = [x for _, x in sorted(zip(algo_position, algo_long_name))]

    algos = algo_ordered
    max_ate = -1
    fig, axes = plt.subplots(nrows=1, ncols=len(platforms), figsize=(18, 5))
    i = 0
    for platform in platforms:
        if platform not in data:
            utils.printerr("DATA FOR PLATFORM MISSING!!")
        MEANS_DATA = []
        LABELS = []
        for algo in algos:
            if algo in data[platform]:
                mean_data = data[platform][algo]
                MEANS_DATA.append(mean_data)
                if np.max(mean_data) > max_ate:
                    max_ate = np.max(mean_data)
                LABELS.append(getlabel(algo))
            else:
                utils.printerr(algo + " not available on " + platform + "!\n")
                # ATE_DATA.append([0])
        pos = [x for x in range(len(LABELS))]

        if len(pos) == 0:
            utils.printerr("Empty data, cannot draw anything.\n")
            return False

        parts = axes[i].violinplot(MEANS_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True,
                                   bw_method='silverman')
        axes[i].set_title(platforms[i] + " ATE", fontsize=20)
        axes[i].set_xticks(pos)
        axes[i].set_xticklabels(LABELS)
        for label in axes[i].get_xmajorticklabels():
            label.set_rotation(30)
            label.set_horizontalalignment("right")

        for pc in parts['bodies']:
            if platforms[i] == "Jetson":
                pc.set_facecolor('#D43F3A')
            elif platforms[i] == "Workstation":
                pc.set_facecolor('#008080')
            pc.set_edgecolor('black')
            pc.set_alpha(0.8)

        i += 1
    for ax in axes:
        ax.set_ylim([0, max_ate + max_ate / 10])

    if filename:
        fig.subplots_adjust(hspace=0.4)
        plt.savefig(filename, bbox_inches='tight')
    else:
        plt.show()

    return True


def plot_violins(data, filename, order):
    violins = data

    ## PLOT DATA ##

    algo_long_name = [x for x in violins.keys() if getlabel(x) in order]
    algo_short_name = [getlabel(x) for x in violins.keys() if getlabel(x) in order]

    algo_position = [order.index(x) for x in algo_short_name]

    algo_ordered = [x for _, x in sorted(zip(algo_position, algo_long_name))]

    algos = algo_ordered
    pos = [x for x in range(len(algos))]

    if len(pos) == 0:
        utils.printerr("Empty data, cannot draw anything.\n")
        return False

    fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(18, 5))

    FPS_DATA = [violins[algo][slamlog.FPS_COLUMN] for algo in algos]
    ATE_DATA = [violins[algo][slamlog.ATE_COLUMN] for algo in algos]
    MEM_DATA = [violins[algo][slamlog.CPU_MEMORY_COLUMN] for algo in algos]

    LABELS = [getlabel(algo) for algo in algos]

    axes[0].violinplot(FPS_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True,
                       bw_method='silverman')
    axes[0].set_title('Speed (FPS)', fontsize=20)

    axes[1].violinplot(ATE_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True,
                       bw_method='silverman')
    axes[1].set_title('Accuracy (ATE in meters)', fontsize=20)

    axes[2].violinplot(MEM_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True,
                       bw_method='silverman')
    axes[2].set_title('Memory CPU (MB)', fontsize=14)

    for ax in axes.flatten():
        ax.set_xticks(pos)
        ax.set_xticklabels(LABELS)

        for label in ax.get_xmajorticklabels():
            label.set_rotation(30)
            label.set_horizontalalignment("right")

    if filename:
        fig.subplots_adjust(hspace=0.4)
        plt.savefig(filename, bbox_inches='tight')
    # else:
    #     plt.show()

    return True


def plot_bars(data, filename, order):
    ## PLOT DATA ##

    algo_long_name = [x for x in data["Laptop"].keys() if getlabel(x) in order]
    algo_short_name = [getlabel(x) for x in data["Laptop"].keys() if getlabel(x) in order]

    algo_position = [order.index(x) for x in algo_short_name]

    algo_ordered = [x for _, x in sorted(zip(algo_position, algo_long_name))]

    algos = algo_ordered
    pos = [x for x in range(len(algos))]

    if len(pos) == 0:
        utils.printerr("Empty data, cannot draw anything.\n")
        return False
    bar_width = 0.15
    fig, axes = plt.subplots(nrows=1, ncols=len(platforms), figsize=(36, 5))

    # labels = ['LR0', 'LR1', 'LR2', 'LR3']
    # labels = ['MH_01', 'MH_02','MH_03','V1_01','V1_02','V2_01','V2_02']
    labels = ['MH_01', 'MH_02','MH_03','V1_01','V1_02','V2_01','V2_02']
    # labels = ["FR1_rpy", "FR2_rpy", "FR1_xyz", "FR2_xyz", "FR1_360", "FR1_desk", "FR1_desk2", "FR1_desk2_p","FR1_floor", "FR1_room", "FR2_desk"]
    widths = np.arange(len(labels))
    algomap = {}
    algocount = 0
    for a in algo_ordered:
        algomap[a] = algocount
        algocount += 1
    print("algomap",algomap)
    # for platform in data.keys():
    plat_no = 0
    max_ate = -1
    for platform, platform_data in sorted(data.items()):
        ax = axes[plat_no]
        ax.set_title(platform)
        ax.set_xticks(widths)
        ax.set_xticklabels(labels)
        for item in ([] + ax.get_xticklabels() + ax.get_yticklabels()):
            item.set_fontsize(10)

        for a, algo_data in platform_data.items():
            current_max = np.max(algo_data)
            if current_max > max_ate:
                max_ate = current_max
            ax.bar(widths + bar_width*algomap[a] - bar_width, algo_data, bar_width, label=getlabel(a))

        ax.yaxis.label.set_size(10)
        ax.xaxis.label.set_size(10)
        plat_no += 1
    # for platform in platforms
    # for
    plot.legend(prop={'size': 10})
    for ax in axes:
        ax.set_ylim([0, max_ate + 0.1])

    if filename:
        fig.subplots_adjust(hspace=0.4)
        plt.savefig(filename, bbox_inches='tight')
    # else:
    #     plt.show()

    return True


def plot_lifelong():
    ## PLOT DATA ##
    # datasets =
    # gt = {}
    # gt["cafe"] = [1708, 2700]
    # gt["office"] = [811, 901, 360, 872, 1591, 1081, 1141]
    # gt["corridor"] = [8513, 3479, 2100, 1890, 4379]
    # gt["market"] = [6145, 7661, 8815]
    # gt["home"] = [4567, 2999, 2639, 2130, 780]
    gt = [1708, 2700, 811, 901, 360, 872, 1591, 1081, 1141, 8513, 3479, 2100, 1890, 4379, 6145, 7661, 8815, 4567, 2999, 2639, 2130, 780]
    # algos = {}
    # efusion = {}
    # efusion["cafe"] = [1408, 1896]
    # efusion["office"] = [811, 901, 360, 872, 1591, 1081, 1141]
    # efusion["corridor"] = [1577, 531, 483, 719, 695]
    # efusion["market"] = [4082, 4184, 2456]
    # efusion["home"] = [1877, 1965, 1948, 1795, 780]
    efusion = [1408, 1896, 811, 901, 360, 872, 1591, 1081, 1141, 1577, 531, 483, 719, 695, 4082, 1911, 377, 1877, 1965,1948, 1795, 780]
    # refusion = {}
    # refusion["cafe"] = [1708, 2481]
    # refusion["office"] = [811, 880, 360, 872, 1591, 1065, 1141]
    # refusion["corridor"] = [1263, 842, 1538, 1668, 1154]
    # refusion["market"] = [4082, 4184, 5970]
    # refusion["home"] = [1059, 1458, 1289, 596, 780]
    refusion = [1708, 2481, 811, 880, 360, 872, 1591, 1065, 1141, 1263, 842, 1538, 1668, 1154, 4082, 4184, 5970, 1059, 1458, 1289, 596, 780]
    # orbslam2 = {}
    # orbslam2["cafe"] = [1708, 1520]
    # orbslam2["office"] = [811, 880, 360, 872, 1591, 1065, 1141]
    # orbslam2["corridor"] = [1376, 494, 1072, 1473, 1501]
    # orbslam2["market"] = [1389, 3131, 499]
    # orbslam2["home"] = [1092, 927, 859, 1540, 780]
    orbslam2 = [1708, 1520, 811, 880, 360, 872, 1591, 1065, 1141, 1376, 494, 1072, 1473, 1501, 1389, 3131, 499, 1092, 927, 859, 1540, 780]
    # orbslam3 = {}
    # orbslam3["cafe"] = [1746, 935]
    # orbslam3["office"] = [811, 880, 360, 872, 1591, 1065, 1141]
    # orbslam3["corridor"] = [1466, 842, 1189, 1890, 1200]
    # orbslam3["market"] = [1425, 3273, 467]
    # orbslam3["home"] = [1601, 1334, 1005, 1227, 780]
    orbslam3 = [1708, 935, 811, 880, 360, 872, 1591, 1065, 1141, 1466, 842, 1189, 1890, 1200, 1425, 3273, 467, 1601, 1334, 1005, 1227, 780]
    # fullfusion = {}
    # fullfusion["cafe"] = [1717, 1720]
    # fullfusion["office"] = [811, 880, 360, 872, 1591, 1065, 1141]
    # fullfusion["corridor"] = [4824, 1789, 1037, 1004, 2421]
    # fullfusion["market"] = [3195, 1911, 377]
    # fullfusion["home"] = [3203, 2099, 1954, 1436, 780]
    fullfusion = [1708, 1720, 811, 880, 360, 872, 1591, 1065, 1141, 4824, 1789, 1037, 1004, 2421, 4184, 3195, 2456, 3203, 2099, 1954, 1436, 780]
    # algos["efusion"] = efusion
    # algos["refusion"] = refusion
    # algos["fullfusion"] = fullfusion
    # algos["orbslam2"] = orbslam2
    # algos["orbslam3"] = orbslam3
    ff = np.array(fullfusion) / np.array(gt) * 100
    orb3 = np.array(orbslam3) / np.array(gt) * 100
    orb2 = np.array(orbslam2) / np.array(gt) * 100
    ref = np.array(refusion) / np.array(gt) * 100
    elastic = np.array(efusion) / np.array(gt) * 100
    print(ff)
    print(orb3)
    print(orb2)
    print(elastic)
    print(ref)
    width = 0.15
    fig, ax = plt.subplots(figsize=(40, 5))

    labels = ["cafe1",
              "cafe2",
              "office1",
              "office2",
              "office3",
              "office4",
              "office5",
              "office6",
              "office7",
              "corridor1",
              "corridor2",
              "corridor3",
              "corridor4",
              "corridor5",
              "market1",
              "market2",
              "market3",
              "home1",
              "home2",
              "home3",
              "home4",
              "home5"]
    x = np.arange(len(labels))
    rects_ff = ax.bar(x - width*2, ff, width, label='FullFusion')
    rects_orb2 = ax.bar(x - width, orb2, width, label='ORB-SLAM2')
    rects_orb3 = ax.bar(x, orb3, width, label='ORB-SLAM3')
    rects_ref = ax.bar(x + width, ref, width, label='ReFusion')
    rects_elastic = ax.bar(x + width*2, elastic, width, label='ElasticFusion')

        # ax.bar(widths + bar_width*algomap[a] - bar_width, algo_data, bar_width, label=getlabel(a))
    def autolabel(rects):
        """Attach a text label above each bar in *rects*, displaying its height."""
        for rect in rects:
            height = rect.get_height()
            if height < 95:
                ax.annotate('{}%'.format(int(height)),
                            xy=(rect.get_x() + rect.get_width() / 2, height),
                            xytext=(0, 3),  # 3 points vertical offset
                            textcoords="offset points",
                            size=8,
                            ha='center', va='bottom')

    autolabel(rects_ff)
    autolabel(rects_orb2)
    autolabel(rects_orb3)
    autolabel(rects_ref)
    autolabel(rects_elastic)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.yaxis.label.set_size(10)
    ax.xaxis.label.set_size(10)
    for item in ([] + ax.get_xticklabels() + ax.get_yticklabels()):
        item.set_fontsize(8)
    # for platform in platforms

    plot.legend(prop={'size': 10})
    plot.show()

    return True

# plot_lifelong()