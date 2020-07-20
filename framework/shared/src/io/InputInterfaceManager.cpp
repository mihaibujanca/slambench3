/*

 Copyright (c) 2020 University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
#include <iostream>
#include <io/FrameBufferSource.h>
#include <io/openni15/ONI15InputInterface.h>
#include <io/openni2/ONI2InputInterface.h>
#include <io/realsense/RealSense2InputInterface.h>
#include <io/InputInterfaceManager.h>

using namespace slambench::io;

InputInterfaceManager::InputInterfaceManager(const std::vector<std::string> &library_filenames) {
#ifdef DO_OPENNI20
    if (library_filenames[0] == "oni2") {
        std::cerr << "Load OpenNI 2 interface ..." << std::endl;
        input_interfaces_.push_back(new slambench::io::openni2::ONI2InputInterface());
        return;
    }
#endif
#ifdef DO_OPENNI15
    if (library_filenames[0] == "oni15") {
            std::cerr << "Load OpenNI 1.5 interface ..." << std::endl;
            input_interfaces_.push_back(new slambench::io::openni15::ONI15InputInterface());
            return;
        }
#endif
#ifdef DO_REALSENSE
    if (library_filenames[0] == "realsense") {
        std::cerr << "Load RealSense interface ..." << std::endl;
        input_interfaces_.push_back(new slambench::io::realsense::RealSense2InputInterface());
        return;
    }
#endif
    // TODO: Handle other types of interface
    // TODO: Add a getFrameStream in Config to handle that
    // TODO: config will be aware of sensors and then sensors will be able to add there arguments
    for(const auto &library_filename : library_filenames) {

        FILE *input_desc = fopen(library_filename.c_str(), "r");
        if (input_desc == nullptr) {
            throw std::logic_error("Could not open the input file");
        }
        auto input_ref = new slambench::io::FileStreamInputInterface(input_desc,
                                                                     new slambench::io::SingleFrameBufferSource());
        input_interfaces_.push_back(input_ref);
        ////workaround to be compatible with benchmarks that does not implement sensors resetting.
        ////assume different input_interface_manager_ has exactly the same types of sensors.
        ////If sensors are different, may introduce problems.
        // FIXME: this works for a single library but doesn't work for more than that. Should be moved inside each library
        //if(!initialized()) {
        //    first_sensors_ = &input_ref->GetSensors();
        //} else {
        //    input_ref->GetSensors() = *first_sensors_;
        //}
    }
}

InputInterface* InputInterfaceManager::GetCurrentInputInterface()
{
    if(input_interfaces_.empty()) {
        throw std::logic_error("Input interface has not been added to SLAM configuration");
    }
    return input_interfaces_.front();
}
// if this is a new dataset, fire sequence end callback
// sequence end callback will trigger a bunch of stuff such as sending the new gt pose in lifelong SLAM, the logger creates a new logging directory, summary is generated (all the below)
SLAMFrame* InputInterfaceManager::GetNextFrame() {
    if (input_stream_ == nullptr) {
        std::cerr << "No input loaded." << std::endl;
        return nullptr;
    }
    if (!input_stream_->HasNextFrame()) {
        input_interfaces_.pop_front();
        auto gt_buffering_stream = new slambench::io::GTBufferingFrameStream(GetCurrentInputInterface()->GetFrames());
        input_stream_ = gt_buffering_stream;
    }



    //input_interface_updated_ = true;
    //current_input_id_++;
    //return true;

    return input_stream_->GetNextFrame();
}

// callback
//TODO: all this in the callback!
//param_manager_.ClearComponents();
//for (slambench::io::Sensor *sensor : GetCurrentInputInterface()->GetSensors()) {
//    param_manager_.AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
//}
//InitGroundtruth();
//InitWriter();
//for (auto lib : slam_libs_) {
//    lib->update_input_interface(GetCurrentInputInterface());
//}

SLAMFrame* InputInterfaceManager::GetClosestGTFrameToTime(slambench::TimeStamp& ts) const {
    return dynamic_cast<slambench::io::GTBufferingFrameStream*>(input_stream_)->GetGTFrames()->GetClosestFrameToTime(ts);
}