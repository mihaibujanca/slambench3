/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef SLAMBENCH_CONFIGURATION_H_
#define SLAMBENCH_CONFIGURATION_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include "ColumnWriter.h"
#include <vector>
#include <string>
#include <chrono>
#include <list>

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <SLAMBenchLibraryHelper.h>

#include <io/sensor/SensorCollection.h>
#include <io/InputInterface.h>
#include <dlfcn.h>

#define LOAD_FUNC2HELPER(handle,lib,f)     *(void**)(& lib->f) = dlsym(handle,#f); const char *dlsym_error_##lib##f = dlerror(); if (dlsym_error_##lib##f) {std::cerr << "Cannot load symbol " << #f << dlsym_error_##lib##f << std::endl; dlclose(handle); exit(1);}

static const unsigned int default_frame_limit                  = 0;
static const double default_realtime_mult                      = 1;
static const std::string default_dump_volume_file              = "";
static const std::string default_log_file                      = "";
static const std::string default_save_map                      = "";
static const std::vector<std::string> default_slam_libraries   = {};
static const std::vector<std::string> default_input_files      = {};
static const bool                     default_is_false         = false;

typedef std::chrono::time_point<std::chrono::high_resolution_clock> stl_time;

class SLAMBenchConfiguration : public ParameterComponent {
private:
    slam_lib_container_t slam_libs;
    std::ofstream log_filestream;
    std::ostream* log_stream;
    std::string   log_file;
    std::vector<std::string> input_files;
    std::vector<std::string> slam_library_names;
    slambench::RowNumberColumn row_number_;
    std::unique_ptr<slambench::ColumnWriter> writer_;
    std::shared_ptr<slambench::metrics::Metric> duration_metric_;
    std::shared_ptr<slambench::metrics::Metric> power_metric_;
    slambench::io::SensorCollection* first_sensors_;
    std::unique_ptr<slambench::outputs::AlignmentOutput> alignment_ = nullptr;

    slambench::io::FrameStream *input_stream_;
    std::list<slambench::io::InputInterface*> input_interfaces_;
    std::vector<std::string> input_filenames_;

    slambench::ParameterManager param_manager_;
    slambench::outputs::OutputManager ground_truth_;

    std::vector<std::function<void()>> frame_callbacks_;
    double realtime_mult_;
    int current_input_id_ = 0;
    unsigned int frame_limit;
    bool initialised_;
    bool realtime_mode_;
    bool gt_available_;
    bool input_interface_updated_ = false;
    bool aided_reloc_ = false;

public:
    SLAMBenchConfiguration(void (*input_callback)(Parameter*, ParameterComponent*) = nullptr,
                           void (*libs_callback)(Parameter*, ParameterComponent*)  = nullptr);
    virtual ~SLAMBenchConfiguration();

    void AddFrameCallback(std::function<void()> callback) { frame_callbacks_.push_back(callback); }
    const slam_lib_container_t &GetLoadedLibs() const { return slam_libs; }
    const slambench::ParameterManager &GetParameterManager() const { return param_manager_; }
    slambench::ParameterManager &GetParameterManager() { return param_manager_; }

    slambench::outputs::OutputManager &GetGroundTruth() { return ground_truth_; }

    /**
     * Initialise the selected libraries and inputs.
     * Initialise the ground truth output manager. All ground truth sensors in
     * the sensor collection are registered as GT outputs, and all frames
     * within the collection are registered as GT output values.
     *
     */
    void InitGroundtruth(bool with_point_cloud = true);

    void InitAlgorithms();

    // Clean up data structures used by algorithms
    void CleanAlgorithms();
    void SaveResults();
    void InitAlignment();
    void InitSensors();
    void InitWriter();
    static void ComputeLoopAlgorithm(SLAMBenchConfiguration *config, bool *stay_on, SLAMBenchUI *ui);

    void AddSLAMLibrary(const std::string& so_file, const std::string &id);
    bool AddInput(const std::string& library_filename);

    const slambench::io::SensorCollection &GetSensors() {

        return GetCurrentInputInterface()->GetSensors();

    }

    void SetInputInterface(slambench::io::InputInterface *input_ref) {
        input_interfaces_.push_front(input_ref);
    }

    void ResetSensors()
    {
        GetParameterManager().ClearComponents();
    }

    inline std::ostream& GetLogStream() {if (!log_stream) UpdateLogStream(); return *log_stream;};
    inline void UpdateLogStream() {
        if (this->log_file != "") {
            this->log_filestream.open(this->log_file.c_str());
            this->log_stream = &(this->log_filestream);
        } else {
            this->log_stream = &std::cout;
        }
    };

    void FireEndOfFrame() { for(auto i : frame_callbacks_) { i(); } }
    void StartStatistics();
    void PrintDse();
    slambench::io::InputInterface *GetCurrentInputInterface();
    void AddInputInterface(slambench::io::InputInterface *input_ref);
    bool LoadNextInputInterface();

    std::string alignment_technique_;
    std::string output_filename_;
};

inline void input_callback(Parameter* param, ParameterComponent* caller) {

    auto config = dynamic_cast<SLAMBenchConfiguration*> (caller);

    if (!config) {
        std::cerr << "Extremely bad usage of the force..." << std::endl;
        std::cerr << "It happened that a ParameterComponent* can not be turned into a SLAMBenchConfiguration*..." << std::endl;
        exit(1);
    }

    auto parameter = dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (const std::string& input_name : parameter->getTypedValue()) {
        config->AddInput(input_name);
    }
    config->InitSensors();
}
inline void help_callback(Parameter*, ParameterComponent* caller) {
    auto config = dynamic_cast<SLAMBenchConfiguration*> (caller);

    std::cerr << " == SLAMBench Configuration ==" << std::endl;
    std::cerr << "  Available parameters :" << std::endl;
    config->GetParameterManager().PrintArguments(std::cerr);

    exit(0);
}

inline void dse_callback(Parameter*, ParameterComponent* caller) {
    auto config = dynamic_cast<SLAMBenchConfiguration*>(caller);
    config->PrintDse();
    exit(0);
}

inline void log_callback(Parameter*, ParameterComponent* caller) {
    auto config = dynamic_cast<SLAMBenchConfiguration*>(caller);
    config->UpdateLogStream();
}

inline void slam_library_callback(Parameter* param, ParameterComponent* caller) {

    auto config = dynamic_cast<SLAMBenchConfiguration*>(caller);
    auto parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (auto &library_name : parameter->getTypedValue()) {

        std::string library_filename;
        std::string library_identifier;

        auto pos = library_name.find("=");
        if (pos != std::string::npos)  {
            library_filename   = library_name.substr(0, pos);
            library_identifier = library_name.substr(pos+1);
        } else {
            library_filename = library_name;
        }
        config->AddSLAMLibrary(library_filename, library_identifier);
    }
}
#endif /* SLAMBENCH_CONFIGURATION_H_ */
