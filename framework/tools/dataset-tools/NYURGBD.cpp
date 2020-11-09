/**
 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "../dataset-tools/include/NYURGBD.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <io/sensor/LabelledCameraSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>


#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <array>

using namespace slambench::io;

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

class DescriptionReader {
    private:
        std::ifstream file;

        int64_t fileTimestamp;
        std::string depthImagePath;
        std::string rgbImagePath;
        std::string acceleratorDPath;
        std::string acceleratorRPath;
        std::string labelledFramePath;

        std::string rootdir;

    public:
        DescriptionReader(const std::string &dirname) {
            size_t pathEnd = dirname.rfind('/');
            if (pathEnd != std::string::npos)
                rootdir = dirname.substr(0, pathEnd + 1);

            file = std::ifstream(dirname + ".txt");
            if (!file) {
                throw "Could not open file: " + dirname + ".txt";
            }
        }

        bool read() {
            return static_cast<bool>(file >> fileTimestamp    >> depthImagePath
                                          >> rgbImagePath     >> acceleratorDPath
                                          >> acceleratorRPath >> labelledFramePath);
        }


        std::string get_rootdir() const {
            return rootdir;
        }

        int64_t get_fileTimestamp() const {
            return fileTimestamp;
        }

        std::string get_depthImagePath() const {
            return depthImagePath;
        }

        std::string get_depthFilledImagePath() const {
            int total = depthImagePath.length();
            std::string depthFilledImagePath = depthImagePath.substr(0, total - 4) + "filled" + depthImagePath.substr(total - 4);
            return depthFilledImagePath;
        }

        std::string get_rgbImagePath() const {
            return rgbImagePath;
        }

        std::string get_acceleratorDPath() const {
            return acceleratorDPath;
        }

        std::string get_acceleratorRPath() const {
            return acceleratorRPath;
        }

        std::string get_labelledFramePath() const {
            return labelledFramePath;
        }
};


bool analyseNYURGBDFolder(const std::string &dirname) {

    try {
        if (!boost::filesystem::exists(dirname))
            return false;
        if (!boost::filesystem::exists(dirname + ".txt"))
            return false;

        DescriptionReader reader(dirname);
        std::string rootdir = reader.get_rootdir();

        while (reader.read()) {
            std::array<std::string, 2> files({{rootdir + "/" + reader.get_depthImagePath(),
                                               rootdir + "/" + reader.get_rgbImagePath()}});

            for (const auto &filename : files) {
                if (!boost::filesystem::exists(filename)) {
                    std::cerr << "File " << filename << " reported at timestamp ";
                    std::cerr << reader.get_fileTimestamp() << " does not exist" << std::endl;
                    return false;
                }
            }
        }

    } catch (boost::filesystem::filesystem_error &e)  {
        std::cerr << "I/O Error with directory " << dirname << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}


bool loadNYURGBDDepthData(const std::string &dirname, SLAMFile &file,
                          const Sensor::pose_t &pose,
                          const DepthSensor::intrinsics_t &intrinsics,
                          const CameraSensor::distortion_coefficients_t &distortion,
                          const DepthSensor::disparity_params_t &disparity_params,
                          const DepthSensor::disparity_type_t &disparity_type) {

    DepthSensor *depth_sensor = new DepthSensor("Depth");
    depth_sensor->Width = 640;
    depth_sensor->Height = 480;
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->Description = "Depth";
    depth_sensor->CopyPose(pose);
    depth_sensor->CopyIntrinsics(intrinsics);
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    depth_sensor->CopyRadialTangentialDistortion(distortion);
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Rate = 30.0;

    file.Sensors.AddSensor(depth_sensor);

    DescriptionReader reader(dirname);
    std::string rootdir = reader.get_rootdir();

    while (reader.read()) {

        int timestampS = reader.get_fileTimestamp() / 1000;
        int timestampNS = reader.get_fileTimestamp() % 1000 * 1000;

        ImageFileFrame *depth_frame = new ImageFileFrame();
        depth_frame->FrameSensor    = depth_sensor;
        depth_frame->Timestamp.S    = timestampS;
        depth_frame->Timestamp.Ns   = timestampNS;

        std::stringstream frame_name;
        frame_name << rootdir << "/" << reader.get_depthImagePath();

        depth_frame->Filename = frame_name.str();

        if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
            printf("No depth image for frame (%s)\n", frame_name.str().c_str());
            perror("");
            return false;
        }

        file.AddFrame(depth_frame);
    }

    return true;
}

bool loadNYURGBDDepthFilledData(const std::string &dirname, SLAMFile &file,
                                const Sensor::pose_t &pose,
                                const DepthSensor::intrinsics_t &intrinsics,
                                const CameraSensor::distortion_coefficients_t &distortion,
                                const DepthSensor::disparity_params_t &disparity_params,
                                const DepthSensor::disparity_type_t &disparity_type) {

    DepthSensor *depth_sensor = new DepthSensor("DepthFilled");
    depth_sensor->Width = 640;
    depth_sensor->Height = 480;
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->Description = "DepthFilled";
    depth_sensor->CopyPose(pose);
    depth_sensor->CopyIntrinsics(intrinsics);
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    depth_sensor->CopyRadialTangentialDistortion(distortion);
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Rate = 30.0;

    file.Sensors.AddSensor(depth_sensor);

    DescriptionReader reader(dirname);
    std::string rootdir = reader.get_rootdir();

    while (reader.read()) {

        int timestampS = reader.get_fileTimestamp() / 1000;
        int timestampNS = reader.get_fileTimestamp() % 1000 * 1000;

        ImageFileFrame *depth_frame = new ImageFileFrame();
        depth_frame->FrameSensor    = depth_sensor;
        depth_frame->Timestamp.S    = timestampS;
        depth_frame->Timestamp.Ns   = timestampNS;

        std::stringstream frame_name;
        frame_name << rootdir << "/" << reader.get_depthFilledImagePath();

        depth_frame->Filename = frame_name.str();

        if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
            printf("No filled depth image for frame (%s)\n", frame_name.str().c_str());
            perror("");
            return false;
        }

        file.AddFrame(depth_frame);
    }

    return true;
}

bool loadNYURGBDRGBData(const std::string &dirname, SLAMFile &file, const Sensor::pose_t &pose,
                        const CameraSensor::intrinsics_t &intrinsics,
                        const CameraSensor::distortion_coefficients_t &distortion) {

    CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    rgb_sensor->Width = 640;
    rgb_sensor->Height = 480;
    rgb_sensor->FrameFormat = frameformat::Raster;
    rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
    rgb_sensor->Description = "RGB";
    rgb_sensor->CopyPose(pose);
    rgb_sensor->CopyIntrinsics(intrinsics);
    rgb_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    rgb_sensor->CopyRadialTangentialDistortion(distortion);
    rgb_sensor->Index = file.Sensors.size();
    rgb_sensor->Rate = 30.0;

    file.Sensors.AddSensor(rgb_sensor);

    DescriptionReader reader(dirname);
    std::string rootdir = reader.get_rootdir();

    while (reader.read()) {

        int timestampS  = reader.get_fileTimestamp() / 1000;
        int timestampNS = reader.get_fileTimestamp() % 1000 * 1000;

        ImageFileFrame *rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor    = rgb_sensor;
        rgb_frame->Timestamp.S    = timestampS;
        rgb_frame->Timestamp.Ns   = timestampNS;

        std::stringstream frame_name;
        frame_name << rootdir << "/" << reader.get_rgbImagePath() ;
        rgb_frame->Filename = frame_name.str();

        if (access(rgb_frame->Filename.c_str(), F_OK) < 0) {
            printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
            perror("");
            return false;
        }

      file.AddFrame(rgb_frame);
    }
    return true;
}

std::map<int, std::string> readLabels(const std::string &rootdir) {
    const std::string filename = rootdir + "/labelNames.txt";

    std::map<int, std::string> map;

    int classNo = 0;
    std::string className;

    std::ifstream f(filename);

    std::string line;
    while (std::getline(f, line)) {
        std::size_t pos = line.find(' ');
        if (pos == std::string::npos) {
            std::cout << "WARNING: Label file invalid!" << std::endl;
            return map;
        }
        classNo = std::stoi(line.substr(0, pos));
        className = line.substr(pos+1);
        std::cout << "Adding " << classNo << " " << className << std::endl;
        map.emplace(classNo, className);
    }

    return map;
}

bool loadNYURGBDLabelledData(const std::string &dirname, SLAMFile &file) {

    constexpr auto pixelformat = pixelformat::D_I_16;

    LabelledCameraSensor *labelled_sensor = new LabelledCameraSensor("Labelled",
                                                                     LabelledCameraSensor::kLabelType);
    labelled_sensor->Index = file.Sensors.size();
    labelled_sensor->Width = 640;
    labelled_sensor->Height = 480;
    labelled_sensor->FrameFormat = frameformat::Raster;
    labelled_sensor->PixelFormat = pixelformat;
    labelled_sensor->Description = "Labelled";

    file.Sensors.AddSensor(labelled_sensor);

    DescriptionReader reader(dirname);
    std::string rootdir = reader.get_rootdir();

    labelled_sensor->labelMap = readLabels(rootdir);

    while (reader.read()) {

        int timestampS  = reader.get_fileTimestamp() / 1000;
        int timestampNS = reader.get_fileTimestamp() % 1000 * 1000;

        ImageFileFrame *labelled_frame = new ImageFileFrame();
        labelled_frame->FrameSensor    = labelled_sensor;
        labelled_frame->Timestamp.S    = timestampS;
        labelled_frame->Timestamp.Ns   = timestampNS;

        if (reader.get_labelledFramePath() == "None") {
            labelled_frame->SetVariableSize(0);
            labelled_frame->Filename = "None";
        } else {
            labelled_frame->SetVariableSize(WIDTH * HEIGHT * pixelformat::GetPixelSize(pixelformat));

            std::stringstream frame_name;
            frame_name << rootdir << "/" << reader.get_labelledFramePath();
            labelled_frame->Filename = frame_name.str();

            if (access(labelled_frame->Filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }
        }

        file.AddFrame(labelled_frame);
    }

    return true;
}


SLAMFile* NYURGBDReader::GenerateSLAMFile () {

    std::cout << "=========================" << std::endl;
    std::cout << "Generating SLAM file" << std::endl;
    std::cout << "=========================" << std::endl;

    if (!(rgb || depth)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;

    if (!analyseNYURGBDFolder(dirname)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    SLAMFile * slamfilep = new SLAMFile();
    SLAMFile & slamfile  = *slamfilep;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();


    DepthSensor::disparity_params_t disparity_params = {0.001, 0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

    std::cout << "STARTING" << std::endl;

    // Load Depth
    if (depth && !loadNYURGBDDepthData(dirname, slamfile, pose,
                                       intrinsics_depth, distortion_depth,
                                       disparity_params, disparity_type)) {
        std::cout << "Error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load Depth Filled
    if (depth_filled && !loadNYURGBDDepthFilledData(dirname, slamfile, pose,
                                                    intrinsics_depth, distortion_depth,
                                                    disparity_params, disparity_type)) {
        std::cout << "Error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load RGB
    if (rgb && !loadNYURGBDRGBData(dirname, slamfile, pose, intrinsics_rgb, distortion_rgb)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load labelled
    if (labels && !loadNYURGBDLabelledData(dirname, slamfile)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    std::cout << "DONE" << std::endl;

    return slamfilep;
}

