//
// Created by mspear on 17/02/20.
//

#ifndef FRAMEWORK_SHARED_INCLUDE_TOML_ALGORITHM_H
#define FRAMEWORK_SHARED_INCLUDE_TOML_ALGORITHM_H

#include "cpptoml.h"

class TOMLAlgorithm {

private:
    std::string _name;
    const std::map<std::string, std::string> stringValues = {
            // orbslam2
            {"mode",        "-m"},
            {"settings",        "-s"},
            {"vocabulary",        "-voc"},
            // efusion
            {"shader_dir",        "-sh"},
    };

    const std::map<std::string, std::string> numberValues = {
            // orbslam2
            {"max_features", "-mf"},
            {"scale_levels", "-sl"},
            {"initial_fast_threshold", "-ift"},
            {"initial_slow_threshold", "-sft"},
            {"fps",          "-fps"},
            {"scale_factor", "-sf"},
            {"dt",          "-dt"},
            // kfusion
            {"compute_size_ratio", "-c"},
            {"integration_rate", "-r"},
            {"tracking_rate", "-t"},
            {"rendering_rate", "-z"},
            {"icp_threshold", "-l"},
            {"mu", "-m"},
            {"pyramid_level1", "-y1"},
            {"pyramid_level2", "-y2"},
            {"pyramid_level3", "-y3"},
            // refusion
            {"voxel_size", "-s"},
            {"truncation_distance", "-td"},
            {"max_depth", "-maxd"},
            {"min_depth", "-mind"},
            {"num_buckets", "-nbuck"},
            {"bucket_size", "-bucks"},
            {"num_blocks", "-nbl"},
            {"block_size", "-bsize"},
            {"max_sdf_weight", "-sdfw"},
            {"max_it_0", "-it0"},
            {"max_it_1", "-it1"},
            {"max_it_2", "-it2"},
            {"reloc", "-ds0"},
            {"fast_0_dom", "-ds1"},
            {"so3", "-ds2"},
            {"min_increment", "-minincr"},
            {"regularisation", "-reg"},
            {"huber", "-hub"},
            {"depth_factor", "-depth"},
            // efusion
            {"confidence", "-c"},
            {"depth", "-d"},
            {"icp", "-icp"},
            {"icp_err_thresh", "-ie"},
            {"cov_thresh", "-cv"},
            {"photo_thresh", "-pt"},
            {"fern_thresh", "-ft"},
            {"icp_count_thresh", "-ic"},
            {"time_delta", "-t"},
            {"texture_dim", "-td"},
            {"node_texture_dim", "-ntd"},
    };

    const std::map<std::string, std::string> arrayValues = {
            // kfusion
            {"volume_size", "-s"},
            {"volume_direction", "-d"},
            {"volume_resolution", "-v"},
    };

    const std::map<std::string, std::string> boolValues = {
            // efusion
            {"open_loop", "-ol"},
            {"reloc", "-rl"},
            {"fast_odom", "-fod"},
            {"so3", "-nso"},
            {"frame_to_frame_rgb", "-ftf"},
    };

public:
    TOMLAlgorithm(std::string name) : _name(name) {};

    void parseConfiguration(const std::shared_ptr<cpptoml::table>& config, std::vector<std::string> &arguments) {

        auto algo = config->get_table(_name);
//        std::cout << "Printing TOML Algorithm ("<< _name << ") Config:" << std::endl;
//        std::cout << (*algo) << "\n\n";

        for (auto const &pair : stringValues) {
            auto value = algo->get_as<std::string>(pair.first);

            if (value) {
                arguments.push_back(pair.second);
                arguments.push_back(*value);
            }
        }

        for (auto const &pair : numberValues) {
            auto value = algo->get_as<double>(pair.first);

            if (value) {
                arguments.push_back(pair.second);
                auto str = std::to_string(*value);
                arguments.push_back(str);
            }
        }

        // take an array of numbers e.g [8, 8, 8] and output comma separated string e.g "8,8,8"
        for (auto const &pair : arrayValues) {
            auto inputs = algo->get_array_of<double>(pair.first);
            if (inputs && inputs->size() > 0) {
                arguments.push_back(pair.second);
                std::string combinedValue = std::to_string((*inputs)[0]);
                if (inputs->size() > 1) {
                    for (size_t i = 1; i < (*inputs).size(); ++i) {
                        combinedValue += ",";
                        combinedValue += std::to_string((*inputs)[i]);
                    }
                }
                arguments.push_back(combinedValue);
            }
        }
    }
};

#endif //FRAMEWORK_SHARED_INCLUDE_TOML_ALGORITHM_H
