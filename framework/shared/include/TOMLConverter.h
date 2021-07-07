//
// Created by mspear on 14/02/20.
//

#ifndef FRAMEWORK_SHARED_INCLUDE_TOML_CONVERTER_H
#define FRAMEWORK_SHARED_INCLUDE_TOML_CONVERTER_H

#include "cpptoml.h"
#include "TOMLInput.h"
#include "TOMLAlgorithm.h"

class TOMLConverter {

private:
    std::string _filename;
    const std::map<std::string, std::string> arrayValues = {
            {"input_files", "-i"},
            {"libraries",   "-load"}
    };
    const std::map<std::string, std::string> stringValues = {
            {"log_file",            "-o"},
            {"alignment_technique", "-a"}
    };
    const std::map<std::string, std::string> numberValues = {
            {"frame_limit",         "-fl"},
            {"log_file",            "-o"},
            {"realtime_multiplier", "-realtime-mult"}
    };
    const std::map<std::string, std::string> boolValues = {
            {"realtime_mode",       "-realtime"},
            {"dse",                 "-dse"}
    };

public:

    void convertTOMLFile(int argc, char* argv[], std::vector<std::string> &arguments) {
        int filename_index = -1;
        bool is_filename_arg = false;

        // copy arguments and extract TOML file
        for (int i = 0; i < argc; ++i) {
            if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--toml") == 0) {
                filename_index = i;
                is_filename_arg = true;
            } else if (is_filename_arg) {
                _filename = argv[i];
                is_filename_arg = false;
            } else {
                arguments.push_back(argv[i]);
            }
        }

        // process and replace TOML arguments
        if (filename_index >= 0) {
            std::cout << "Found TOML file at "<< _filename << std::endl;

            std::vector<std::string> toml_arguments;
            parseConfiguration(toml_arguments);

            // Display the converted command line output
            for (const auto item : toml_arguments) {
                std::cout << item << " ";
            }
            std::cout << std::endl;

            arguments.insert(arguments.begin() + filename_index, toml_arguments.begin(), toml_arguments.end());
        }
    }

    void parseConfiguration(std::vector<std::string> &arguments) {

        try {
            auto config = cpptoml::parse_file(this->_filename);
            std::cout << "Printing TOML Config:" << std::endl;
            std::cout << (*config) << "\n\n";

            for (auto const &pair : stringValues) {
                auto value = config->get_as<std::string>(pair.first);
                std::cout << pair.first << " " << *value << std::endl;

                if (value) {
                    arguments.push_back(pair.second);
                    arguments.push_back(*value);
                }
            }

            for (auto const &pair : numberValues) {
                auto value = config->get_as<double>(pair.first);

                if (value) {
                    arguments.push_back(pair.second);
                    auto str = std::to_string(*value);
                    arguments.push_back(str);
                }
            }

            for (auto const &pair : boolValues) {
                auto value = config->get_as<bool>(pair.first);

                if (value && (*value)) {
                    arguments.push_back(pair.second);
                }
            }

            auto inputs = config->get_array_of<std::string>("input_files");
            auto input_names = config->get_array_of<std::string>("input_names");

            if (inputs) {
                arguments.push_back("-i");
                for (size_t i = 0; i < (*inputs).size(); ++i) {
                    arguments.push_back((*inputs)[i]);

                    // check if input name was provided, only parse if a name is provided for each library
                    if (input_names && inputs->size() == input_names->size()) {
                        auto name = (*input_names)[i];
                        auto input = TOMLInput(name);
                        input.parseConfiguration(config, arguments);
                    }
                }
            }

            auto libraries = config->get_array_of<std::string>("libraries");
            auto library_names = config->get_array_of<std::string>("library_names");

            if (libraries) {
                arguments.push_back("-load");

                for (size_t i = 0; i < (*libraries).size(); ++i) {
                    arguments.push_back((*libraries)[i]);

                    // check if library name was provided, only parse if a name is provided for each library
                    if (library_names && libraries->size() == library_names->size()) {
                        auto name = (*library_names)[i];
                        auto algorithm = TOMLAlgorithm(name);
                        algorithm.parseConfiguration(config, arguments);
                    }
                }
            }

        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

};

#endif //FRAMEWORK_SHARED_INCLUDE_TOML_CONVERTER_H
