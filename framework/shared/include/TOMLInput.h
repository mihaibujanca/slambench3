//
// Created by mspear on 17/02/20.
//

#ifndef FRAMEWORK_SHARED_INCLUDE_TOML_INPUT_H
#define FRAMEWORK_SHARED_INCLUDE_TOML_INPUT_H

class TOMLInput {

private:
    std::string _name;
    const std::map<std::string, std::string> parameterValues = {
            {"depth_intrinsics", "-Depth-ip"},
            {"depth_disparity", "-Depth-dip"},
            {"grey_intrinsics", "-Grey-ip"},
            {"rgb_intrinsics", "-RGB-ip"}
    };

public:
    TOMLInput(std::string name) : _name(name) {};

    void parseConfiguration(const std::shared_ptr<cpptoml::table>& config, std::vector<std::string> &arguments) {

        auto input = config->get_table(_name);
//        std::cout << "Printing TOML Input ("<< _name << ") Config:" << std::endl;
//        std::cout << (*input) << "\n\n";

        // take an array of numbers e.g [8, 8, 8] and output comma separated string e.g "8,8,8"
        for (auto const &pair : parameterValues) {
            auto inputs = input->get_array_of<double>(pair.first);
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

#endif //FRAMEWORK_SHARED_INCLUDE_TOML_INPUT_H
