/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "SLAMBenchConfiguration.h"
#include "TimeStamp.h"
#include <Parameters.h>
#include "sb_malloc.h"

#include <io/FrameBufferSource.h>
#include <io/openni2/ONI2FrameStream.h>
#include <io/openni2/ONI2InputInterface.h>
#include <io/openni15/ONI15FrameStream.h>
#include <io/openni15/ONI15InputInterface.h>

#include <io/InputInterface.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>

#include <outputs/TrajectoryAlignmentMethod.h>
#include <outputs/OutputManagerWriter.h>
#include <metrics/DurationMetric.h>
#include <metrics/PowerMetric.h>
#include <metrics/Metric.h>
#include <metrics/MemoryMetric.h>
#include <metrics/ATEMetric.h>
#include <metrics/RPEMetric.h>
#include <metrics/DepthEstimationMetric.h>

#include <values/Value.h>
#include <outputs/Output.h>

#include <stdexcept>
#include <map>
#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <memory>
#include <assert.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "ResultWriter.h"

SLAMBenchConfiguration::SLAMBenchConfiguration(void (*custom_input_callback)(Parameter*, ParameterComponent*),void (*libs_callback)(Parameter*, ParameterComponent*)) :
        ParameterComponent("") , input_stream_(nullptr)  {

    if(!custom_input_callback)
        custom_input_callback = input_callback;
    if(!libs_callback)
        libs_callback = slam_library_callback;
    initialised_ = false;
    log_stream_ = nullptr;
    slam_library_names_ = {};
    // Run Related
    addParameter(TypedParameter<unsigned int>("fl", "frame-limit", "last frame to compute", &frame_limit_, &default_frame_limit));
    addParameter(TypedParameter<unsigned int>("s", "start-frame", "first frame to compute", &start_frame_, &default_start_frame));
    addParameter(TypedParameter<std::string>("o", "log-file", "Output log file", &log_file_, &default_log_file, log_callback));
    addParameter(TypedParameter<std::vector<std::string>>("i", "input" , "Specify the input file or mode." , &input_files_, &default_input_files , custom_input_callback));
    addParameter(TypedParameter<std::vector<std::string> >("load", "load-slam-library" , "Load a specific SLAM library."     , &slam_library_names_, &default_slam_libraries, libs_callback));
    addParameter(TriggeredParameter("dse",   "dse",    "Output solution space of parameters.",    dse_callback));
    addParameter(TriggeredParameter("h",     "help",   "Print the help.", help_callback));
    addParameter(TypedParameter<bool>("realtime",     "realtime-mode",      "realtime frame loading mode",                   &realtime_mode_, &default_is_false));
    addParameter(TypedParameter<double>("realtime-mult",     "realtime-multiplier",      "realtime frame loading mode",                   &realtime_mult_, &default_realtime_mult));

    param_manager_.AddComponent(this);
}

SLAMBenchConfiguration::~SLAMBenchConfiguration()
{
    CleanAlgorithms();
}

void SLAMBenchConfiguration::AddSLAMLibrary(const std::string& so_file, const std::string &id) {

    std::cerr << "new library name: " << so_file  << std::endl;

    void* handle = dlopen(so_file.c_str(),RTLD_LAZY);

    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << std::endl;
        exit(1);
    }
    auto libname_start = so_file.find_last_of('/')+3;
    auto libName = so_file.substr(libname_start, so_file.find('-', libname_start));
    auto lib_ptr = new SLAMBenchLibraryHelper(id, libName, this->GetLogStream(), this->GetCurrentInputInterface());
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_init_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_new_slam_configuration);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_frame);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_process_once);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_clean_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_outputs);
    // workaround to be compatible with benchmarks that does not implement the relocalize API
    if (dlsym(handle, "_Z13sb_relocalizeP22SLAMBenchLibraryHelper")) {
        LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_relocalize);
    } else {
        std::cout << "Benchmark does not implement sb_relocalize(). Will use the default." << std::endl;
        lib_ptr->c_sb_relocalize = lib_ptr->c_sb_process_once;
    }
    slam_libs_.push_back(lib_ptr);


    size_t pre = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    if (!lib_ptr->c_sb_new_slam_configuration(lib_ptr)) {
        std::cerr << "Configuration construction failed." << std::endl;
        exit(1);
    }
    size_t post = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    std::cerr << "Configuration consumed " << post-pre  << " bytes" << std::endl;

    param_manager_.AddComponent(lib_ptr);

    std::cerr << "SLAM library loaded: " << so_file << std::endl;
}

bool SLAMBenchConfiguration::AddInput(const std::string& library_filename) {

    // TODO: Handle other types of interface
    // TODO: Add a getFrameStream in Config to handle that
    // TODO: config will be aware of sensors and then sensors will be able to add there arguments
    if (library_filename == "oni2") {
        std::cerr << "Load OpenNI 2 interface ..." << std::endl;
        this->SetInputInterface(new slambench::io::openni2::ONI2InputInterface());
    } else if (library_filename == "oni15") {
        std::cerr << "Load OpenNI 1.5 interface ..." << std::endl;
        this->SetInputInterface(new slambench::io::openni15::ONI15InputInterface());
    } else {
        FILE * input_desc = fopen(library_filename.c_str(), "r");
        if (input_desc == nullptr) {
            throw std::logic_error( "Could not open the input file" );
        }
        AddInputInterface(new slambench::io::FileStreamInputInterface(input_desc, new slambench::io::SingleFrameBufferSource()));
        input_filenames_.push_back(library_filename);
    }
    return true;
}

void SLAMBenchConfiguration::PrintDse() {
    for (SLAMBenchLibraryHelper* lib : slam_libs_) {
        std::cout << "libs:" << lib->GetIdentifier() << "\n" ;
        for (auto parameter : lib->getParameters()) {
            std::cout << "argument:" << parameter->getLongOption(lib) << "\n" ;
            std::cout << parameter->getStrDetails(lib) << "\n" ;
        }
    }
    exit(0);
}

void SLAMBenchConfiguration::StartStatistics() {

    GetLogStream().setf(std::ios::fixed, std::ios::floatfield);
    GetLogStream().precision(10);

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo=localtime(&rawtime);
    strftime(buffer,80,"%Y-%m-%d %I:%M:%S",timeinfo);
    this->GetLogStream() << "SLAMBench Report run started:\t" << buffer << std::endl << std::endl;

    // Print arguments known so far

    this->GetLogStream() << "Properties:" << std::endl << "=================" << std::endl << std::endl;

    param_manager_.PrintValues(GetLogStream());
}

void SLAMBenchConfiguration::InitGroundtruth(bool with_point_cloud) {
    if(initialised_) {
        //return;
        // auto gt_trajectory = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    }
    auto interface = GetCurrentInputInterface();
    if(interface != nullptr) {
        auto gt_buffering_stream = new slambench::io::GTBufferingFrameStream(interface->GetFrames());
        input_stream_ = gt_buffering_stream;

        if(realtime_mode_) {
            std::cerr << "Real time mode enabled" << std::endl;
            input_stream_ = new slambench::io::RealTimeFrameStream(input_stream_, realtime_mult_, true);
        } else {
            std::cerr << "Process every frame mode enabled" << std::endl;
        }

        ground_truth_.LoadGTOutputsFromSLAMFile(interface->GetSensors(), gt_buffering_stream->GetGTFrames(), with_point_cloud);
    }

    auto gt_trajectory = ground_truth_.GetMainOutput(slambench::values::VT_POSE);
    if(gt_trajectory == nullptr) {
        // Warn if there is no ground truth
        std::cerr << "Dataset does not provide a GT trajectory" << std::endl;
    }

    initialised_ = true;
}

void SLAMBenchConfiguration::InitAlgorithms() {

    assert(initialised_);

    for (auto &lib : slam_libs_) {

        bool init_worked = lib->c_sb_init_slam_system(lib) ;
        //lib->GetMetricManager().EndInit();

        if (!init_worked) {
            std::cerr << "Algorithm initialization failed." << std::endl;
            exit(1);
        }

        auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if(trajectory == nullptr) {
            std::cerr << "Algo does not provide a main pose output" << std::endl;
            exit(1);
        }
    }
    InitWriter();
}

void SLAMBenchConfiguration::ComputeLoopAlgorithm(bool *stay_on, SLAMBenchUI *ui) {

    assert(initialised_);

    // If no trajectory warn and disable trajectory metrics.
    for (auto lib : slam_libs_) {
        auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if (trajectory == nullptr) {
            std::cerr << "Algorithm does not provide a main pose output" << std::endl;
            exit(1);
        }
    }

    int input_seq = 0;
    bool ongoing = false;
    std::map<SLAMBenchLibraryHelper*, Eigen::Matrix4f> libs_trans;

    // ********* [[ MAIN LOOP ]] *********
    while(true) {
        unsigned int frame_count = 0;
        if (ui) {
            if (frame_count != 0 && !ui->IsFreeRunning()) {
                if (!ongoing) {
                    if (!ui->WaitForFrame()) {
                        std::cerr << "!ui->WaitForFrame() ==> break" << std::endl;
                        break;
                    }
                }
            }
        }

        // ********* [[ LOAD A NEW FRAME ]] *********
        if (input_stream_ == nullptr) {
            std::cerr << "No input loaded." << std::endl;
            break;
        }

        auto current_frame = input_stream_->GetNextFrame();

        while (current_frame != nullptr) {
            frame_count++;
            if (current_frame->FrameSensor->GetType() != slambench::io::GroundTruthSensor::kGroundTruthTrajectoryType) {
                // ********* [[ NEW FRAME PROCESSED BY ALGO ]] *********

                for (auto lib : slam_libs_) {
                    // ********* [[ SEND THE FRAME ]] *********
                    ongoing = not lib->c_sb_update_frame(lib, current_frame);

                    // This algorithm hasn't received enough frames yet.
                    if (ongoing) {
                        continue;
                    }

                    // ********* [[ PROCESS ALGO START ]] *********
                    lib->GetMetricManager().BeginFrame();
                    slambench::TimeStamp ts = current_frame->Timestamp;

                    if (!input_interface_updated_) {
                        if (not lib->c_sb_process_once (lib)) {
                            std::cerr <<"Error after lib->c_sb_process_once." << std::endl;
                            exit(1);
                        }
                    } else {
                        // ********** [[or relocalization]] **********
                        //Mihai: need assertion / safety mechanism to avoid ugly errors
                        bool res = lib->c_sb_relocalize(lib);
                        input_interface_updated_ = false;
                        /* If the library failed to re-localize at the beginning of a new input,
                           the framework will send a ground-truth pose to it (so-called aided_reloc).
                           The sent pose is transformed to be relative to the first estimated pose
                           from the library. */
                        if(!res && gt_available_)
                        {
                            aided_reloc_ = true;
                            //Find the nearest one
                            auto gt_frame = dynamic_cast<slambench::io::GTBufferingFrameStream*>(input_stream_)->GetGTFrames()->GetClosestFrameToTime(ts);
                            Eigen::Matrix4f &t = libs_trans[lib];
                            Eigen::Matrix4f gt;
                            memcpy(gt.data(), gt_frame->GetData(), gt_frame->GetSize());
                            dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();
                            Eigen::Matrix4f es = t.inverse() * gt;
                            memcpy(gt_frame->GetData(), es.data(), gt_frame->GetSize());
                            dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();

                            lib->c_sb_update_frame(lib, gt_frame);// groundtruth feed

                            dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();
                            memcpy(gt_frame->GetData(), gt.data(), gt_frame->GetSize());
                            dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();
                        }
                    }

                    if (!lib->c_sb_update_outputs(lib, &ts)) {
                        std::cerr << "Failed to get outputs" << std::endl;
                        exit(1);
                    }

                    lib->GetMetricManager().EndFrame();
                    if (libs_trans.count(lib) == 0 && gt_available_) {
                        libs_trans[lib] = alignment_->getTransformation();
                    }
                }
                // ********* [[ FINALIZE ]] *********
                if (!ongoing) {
                    FireEndOfFrame();
                    if (ui) ui->stepFrame();
                    frame_count += 1;

                    if (frame_limit_ > 0 && frame_count >= frame_limit_) {
                            break;
                    }
                }
            }
            current_frame->FreeData();
            current_frame = input_stream_->GetNextFrame();
        //    TODO: input_stream_manager->GetNextFrame()
        //inside GetNextFrame: if last this is a new dataset, fire sequence end callback
        // sequence end callback will trigger a bunch of stuff such as sending the new gt pose in lifelong SLAM, the logger creates a new logging directory, summary is generated (all the below)
        } // we're done with the frame
        if (!output_filename_.empty()) SaveResults();
        // Load next input if there be
        if (!LoadNextInputInterface()) break;
        // Freeze the alignment after end of the first input
        if (input_seq++ == 0) alignment_->SetFreeze(true);
    }
}

void SLAMBenchConfiguration::CleanAlgorithms()
{
    for (auto lib : slam_libs_) {

        std::cerr << "Clean SLAM system ..." << std::endl;
        bool clean_worked = lib->c_sb_clean_slam_system();

        if (!clean_worked) {
            std::cerr << "Algorithm cleaning failed." << std::endl;
            exit(1);
        } else {
            std::cerr << "Algorithm cleaning succeed." << std::endl;
        }
    }
}

//FIXME: save results in TUM format
void SLAMBenchConfiguration::SaveResults()
{
    if (output_filename_ == "" ) {
        return;
    }
    boost::filesystem::path input_name(input_filenames_[current_input_id_]);
    input_name = input_name.filename();
    boost::filesystem::path output_name = boost::filesystem::path(output_filename_);
    boost::filesystem::path output_prefix;
    boost::filesystem::path gt_dir;
    if (boost::filesystem::is_directory(output_name)) {
        output_name.append("/");
        output_prefix = output_name;
        gt_dir = output_name;
    } else {
        output_prefix = output_name.replace_extension().append("-");
        gt_dir = output_name.parent_path();
    }
    boost::filesystem::path gt_file = gt_dir;
    gt_file += input_name;
    gt_file.replace_extension(".gt");

    bool first_input = current_input_id_ == 0;
    static std::string cpu_info = ResultWriter::GetCPUModel();
    static std::string gpu_info = "";// memory_metric->cuda_monitor.IsActive() ? memory_metric->cuda_monitor.device_name : "";
    static std::string mem_info = ResultWriter::GetMemorySize();

    for(SLAMBenchLibraryHelper *lib : slam_libs_) {
        std::string filename = output_prefix.append(lib->GetLibraryName() + ".txt").string();
        std::ofstream out(filename, first_input ? std::ios::out : std::ios::app);
        ResultWriter writer(out);
        if (first_input) {
            writer.WriteKV("benchmark", lib->GetLibraryName());
            writer.WriteKV("inputs", input_filenames_);
            writer.WriteKV("CPU", cpu_info);
            if (!gpu_info.empty()) writer.WriteKV("GPU", gpu_info);
            writer.WriteKV("memory", mem_info);
        }
        out << std::endl;
        writer.WriteKV("input", input_name.string());
        writer.WriteKV("aided_reloc", std::to_string(aided_reloc_));
        slambench::outputs::BaseOutput::value_map_t traj = lib->GetOutputManager().GetOutput("Pose")->GetValues();
        writer.WriteTrajectory(traj);
        std::cout << "Results saved into " << filename << std::endl;
    }

    if (!boost::filesystem::exists(gt_file)) {
        std::ofstream out(gt_file.string());
        slambench::outputs::BaseOutput::value_map_t traj = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE)->GetValues();
        ResultWriter writer(out);
        writer.WriteKV("input", input_name.string());
        writer.WriteTrajectory(traj);
        std::cout << "Ground-truth saved into " << gt_file.string() << std::endl;
    }
}

void SLAMBenchConfiguration::InitAlignment() {
    slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
    if(alignment_technique_ == "original") {
        alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
    } else if(alignment_technique_ == "new") {
        alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
    } else if(alignment_technique_ == "umeyama") {
        alignment_method = new slambench::outputs::UmeyamaTrajectoryAlignmentMethod();
    } else {
        std::cerr << "Unknown alignment method " << alignment_technique_ << std::endl;
        throw std::logic_error("Unknown alignment method");
    }

    auto gt_traj = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);

    assert(slam_libs_.size() == 1); // following code cannot work with multiple libs
    SLAMBenchLibraryHelper *lib = slam_libs_.front();
    auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
    alignment_.reset(new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj), lib_traj, alignment_method));
    alignment_->SetActive(true);
    alignment_->SetKeepOnlyMostRecent(true);

    //Align point cloud
    auto pointcloud = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD);
    if(pointcloud != nullptr) {
        auto pc_aligned = new slambench::outputs::AlignedPointCloudOutput(pointcloud->GetName() + "(Aligned)", alignment_.get(), pointcloud);
        lib->GetOutputManager().RegisterOutput(pc_aligned);
    }

}

void SLAMBenchConfiguration::InitWriter() {
    if (writer_) {
        for(SLAMBenchLibraryHelper *lib : slam_libs_) {
            lib->GetMetricManager().reset();
            lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->reset();
        }
        aided_reloc_ = false;
    } else {
        // the following metrics last for all inputs; other metrics are allocated for only one input
        duration_metric_ = std::make_shared<slambench::metrics::DurationMetric>();
        power_metric_ = std::make_shared<slambench::metrics::PowerMetric>();
    }
    auto gt_traj = ground_truth_.GetMainOutput(slambench::values::VT_POSE);

    if (!alignment_) InitAlignment();
    writer_.reset(new slambench::ColumnWriter(GetLogStream(), "\t"));
    writer_->AddColumn(&(row_number_));
    bool have_timestamp = false;

    for(SLAMBenchLibraryHelper *lib : slam_libs_) {

        // retrieve the trajectory of the lib
        auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if (lib_traj == nullptr) {
            std::cerr << "There is no output trajectory in the library outputs." << std::endl;
            exit(1);
        }

        // Create timestamp column if we don't have one
        if(!have_timestamp) {
            have_timestamp = true;
            writer_->AddColumn(new slambench::OutputTimestampColumnInterface(lib_traj));
        }

        if (gt_traj) {
            gt_available_ = true;
            // Create an aligned trajectory
            auto aligned = new slambench::outputs::AlignedPoseOutput(lib_traj->GetName() + " (Aligned)", &*alignment_, lib_traj);
            //lib->GetOutputManager().RegisterOutput(aligned);

            // Add ATE metric
            auto ate_metric = std::make_shared<slambench::metrics::ATEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
            if (ate_metric->GetValueDescription().GetStructureDescription().size() > 0) {
                lib->GetMetricManager().AddFrameMetric(ate_metric);
                writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*ate_metric, lib->GetMetricManager().GetFramePhase()));
            }

            // Add RPE metric
            auto rpe_metric = std::make_shared<slambench::metrics::RPEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
            lib->GetMetricManager().AddFrameMetric(rpe_metric);
            writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*rpe_metric, lib->GetMetricManager().GetFramePhase()));

          } else {
            gt_available_ = false;
        }

        // Add a duration metric
        lib->GetMetricManager().AddFrameMetric(duration_metric_);
        lib->GetMetricManager().AddPhaseMetric(duration_metric_);
        writer_->AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric_, lib->GetMetricManager().GetFramePhase()));

        for(auto phase : lib->GetMetricManager().GetPhases()) {
            writer_->AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric_, phase));
        }

        // Add a memory metric
        auto memory_metric = std::make_shared<slambench::metrics::MemoryMetric>();
        lib->GetMetricManager().AddFrameMetric(memory_metric);
        writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*memory_metric, lib->GetMetricManager().GetFramePhase()));

        // Add a power metric if it makes sense
        if (power_metric_->GetValueDescription().GetStructureDescription().size() > 0) {
            lib->GetMetricManager().AddFrameMetric(power_metric_);
            writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*power_metric_, lib->GetMetricManager().GetFramePhase()));
        }
//            FIXME: workaround for ground truth
//            auto depth_est_output = lib->GetOutputManager().GetOutput("depth_est");
//            auto depth_est_gt = lib->GetOutputManager().GetOutput("depth_gt");
//            if(!(depth_est_output&&depth_est_gt)) {
//                std::cerr<<"This algorithm does not provide depth estimation"<<std::endl;
//            }
//            else {
//                auto depth_metric = new slambench::metrics::DepthEstimationMetric(depth_est_output,depth_est_gt);
//                lib->GetMetricManager().AddFrameMetric(depth_metric);
//                cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, depth_metric, lib->GetMetricManager().GetFramePhase()));
//            }
        // Add XYZ row from the trajectory
        auto traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        traj->SetActive(true);
        writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, new slambench::outputs::PoseToXYZOutput(traj)));
    }

    frame_callbacks_.clear();
    AddFrameCallback([this]{writer_->PrintRow();}); // @suppress("Invalid arguments")
    writer_->PrintHeader();
}

void SLAMBenchConfiguration::AddInputInterface(slambench::io::InputInterface *input_ref) {
    //workaround to be compatible with benchmarks that does not implement sensors resetting.
    //assume different input_interfaces_ has exactly the same types of sensors.
    //If sensors are different, may introduce problems.
    if (input_interfaces_.empty()) {
        first_sensors_ = &input_ref->GetSensors();
    } else {
        input_ref->GetSensors() = *first_sensors_;
    }
    input_interfaces_.push_back(input_ref);
}

slambench::io::InputInterface* SLAMBenchConfiguration::GetCurrentInputInterface()
{
    if(input_interfaces_.empty()) {
        throw std::logic_error("Input interface has not been added to SLAM configuration");
    }
    return input_interfaces_.front();
}

void SLAMBenchConfiguration::InitSensors() {
    for (slambench::io::Sensor *sensor : GetCurrentInputInterface()->GetSensors()) {
        param_manager_.AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
    }
}

bool SLAMBenchConfiguration::LoadNextInputInterface() {
    input_interfaces_.pop_front();
    ResetSensors();
    if(input_interfaces_.empty())
        return false;

    InitSensors();
    InitGroundtruth();
    InitWriter();
    for (auto lib : slam_libs_) {
        lib->update_input_interface(GetCurrentInputInterface());
    }
    input_interface_updated_ = true;
    current_input_id_++;
    return true;
}