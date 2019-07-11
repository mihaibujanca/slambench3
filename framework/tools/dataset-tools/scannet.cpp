/**
 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "../dataset-tools/include/scannet.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/LabelledPointCloudSensor.h>
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

std::string getDepthFilename(int frame_number, const std::string &dirname) {
    std::stringstream depth_frame;
    depth_frame << dirname << "/depth/frame-" << std::setfill('0') << std::setw(6) << frame_number << ".depth.png.png";
    return depth_frame.str();
}

std::string getRgbFilename(int frame_number, const std::string &dirname) {
    std::stringstream rgb_frame;
    rgb_frame << dirname << "/rgb/frame-" << std::setfill('0') << std::setw(6) << frame_number << ".color.png";
    return rgb_frame.str();
}

std::string getLabelFilename(int frame_number, const std::string &dirname) {
    std::stringstream labels_frame;
    labels_frame << dirname << "/labels/" << frame_number << ".png";
    return labels_frame.str();
}


/**
 * @param frame_number Will store the highest valid frame number plus 1
 */
bool analyseScannetFolder(const std::string &dirname, int &frame_number) {

    try {
        if (!boost::filesystem::exists(dirname)) {
            std::cout << "Directory " << dirname << " does not exist" << std::endl;
            return false;
        }
        if (!boost::filesystem::exists(dirname + "/depth")) {
            std::cout << "Directory " << dirname << "/depth does not exist" << std::endl;
            return false;
        }
        if (!boost::filesystem::exists(dirname + "/rgb")) {
            std::cout << "Directory " << dirname << "/rgb does not exist" << std::endl;
            return false;
        }
        if (!boost::filesystem::exists(dirname + "/labels")) {
            std::cout << "Directory " << dirname << "/labels does not exist" << std::endl;
            return false;
        }

        frame_number = 0;

        do {
            const std::string depth_filename = getDepthFilename(frame_number, dirname);
            const std::string rgb_filename = getRgbFilename(frame_number, dirname);
            const std::string label_filename = getLabelFilename(frame_number, dirname);

            if (!boost::filesystem::exists(depth_filename)) {
                std::cout << depth_filename << " does not exist" << std::endl;
                return true;
            }
            if (!boost::filesystem::exists(rgb_filename)) {
                std::cout << rgb_filename << " does not exist" << std::endl;
                return true;
            }
            if (!boost::filesystem::exists(label_filename)) {
                std::cout << label_filename << " does not exist" << std::endl;
                return true;
            }

            frame_number++;
        } while(1);


    } catch (boost::filesystem::filesystem_error &e)  {
        std::cerr << "I/O Error with directory " << dirname << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}


bool loadScannetDepthData(const std::string &dirname, SLAMFile &file,
                          const Sensor::pose_t &pose,
                          const DepthSensor::intrinsics_t &intrinsics,
                          const CameraSensor::distortion_coefficients_t &distortion,
                          const DepthSensor::disparity_params_t &disparity_params,
                          const DepthSensor::disparity_type_t &disparity_type,
                          const int noOfFrames) {

    constexpr int rate = 30;

    DepthSensor *depth_sensor = new DepthSensor("Depth");
    depth_sensor->Width = 640;
    depth_sensor->Height = 478;
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
    depth_sensor->Rate = rate;

    file.Sensors.AddSensor(depth_sensor);

    for (int64_t frame_number = 0, timestampMS = 0; frame_number < noOfFrames; frame_number++, timestampMS += 1000 / rate) {
        auto timestampS  = timestampMS / 1000;
        auto timestampNS = timestampMS % 1000 * 1000;

        ImageFileFrame *depth_frame = new ImageFileFrame();
        depth_frame->FrameSensor    = depth_sensor;
        depth_frame->Timestamp.S    = timestampS;
        depth_frame->Timestamp.Ns   = timestampNS;

        depth_frame->Filename = getDepthFilename(frame_number, dirname);
        if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
            printf("No depth image for frame (%s)\n", depth_frame->Filename.c_str());
            perror("");
            return false;
        }

        file.AddFrame(depth_frame);
    }

    return true;
}

bool loadScannetRGBData(const std::string &dirname, SLAMFile &file, const Sensor::pose_t &pose,
                        const CameraSensor::intrinsics_t &intrinsics,
                        const CameraSensor::distortion_coefficients_t &distortion,
                        const int noOfFrames) {

    constexpr int rate = 30;

    CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    rgb_sensor->Width = 640;
    rgb_sensor->Height = 478;
    rgb_sensor->FrameFormat = frameformat::Raster;
    rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
    rgb_sensor->Description = "RGB";
    rgb_sensor->CopyPose(pose);
    rgb_sensor->CopyIntrinsics(intrinsics);
    rgb_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    rgb_sensor->CopyRadialTangentialDistortion(distortion);
    rgb_sensor->Index = file.Sensors.size();
    rgb_sensor->Rate = rate;

    file.Sensors.AddSensor(rgb_sensor);

    for (int64_t frame_number = 0, timestampMS = 0; frame_number < noOfFrames; frame_number++, timestampMS += 1000 / rate) {
        auto timestampS  = timestampMS / 1000;
        auto timestampNS = timestampMS % 1000 * 1000;

        ImageFileFrame *rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor    = rgb_sensor;
        rgb_frame->Timestamp.S    = timestampS;
        rgb_frame->Timestamp.Ns   = timestampNS;

        rgb_frame->Filename = getRgbFilename(frame_number, dirname);
        if (access(rgb_frame->Filename.c_str(), F_OK) < 0) {
            printf("No depth image for frame (%s)\n", rgb_frame->Filename.c_str());
            perror("");
            return false;
        }

        file.AddFrame(rgb_frame);
    }

    return true;
}

std::map<int, std::string> ScannetReader::readLabels(const std::string &dirname) {
    const std::string filename = dirname + "/labelNames.txt";

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

bool ScannetReader::loadScannetLabelledData(const std::string &dirname,
                                            SLAMFile &file,
                                            const int noOfFrames) {

    constexpr auto pixelformat = pixelformat::D_I_16;
    constexpr int rate = 30;

    LabelledCameraSensor *labelled_sensor = new LabelledCameraSensor("Labelled",
                                                                     LabelledCameraSensor::kLabelType);
    labelled_sensor->Index = file.Sensors.size();
    labelled_sensor->Width = 640;
    labelled_sensor->Height = 478;
    labelled_sensor->FrameFormat = frameformat::Raster;
    labelled_sensor->PixelFormat = pixelformat;
    labelled_sensor->Description = "Labelled";

    file.Sensors.AddSensor(labelled_sensor);

    labelled_sensor->labelMap = readLabels(dirname);

    for (int64_t frame_number = 0, timestampMS = 0; frame_number < noOfFrames; frame_number++, timestampMS += 1000 / rate) {
        auto timestampS  = timestampMS / 1000;
        auto timestampNS = timestampMS % 1000 * 1000;

        ImageFileFrame *labelled_frame = new ImageFileFrame();
        labelled_frame->FrameSensor    = labelled_sensor;
        labelled_frame->Timestamp.S    = timestampS;
        labelled_frame->Timestamp.Ns   = timestampNS;
        labelled_frame->SetVariableSize(labelled_sensor->Width * labelled_sensor->Height * pixelformat::GetPixelSize(pixelformat));

        labelled_frame->Filename = getLabelFilename(frame_number, dirname);
        if (access(labelled_frame->Filename.c_str(), F_OK) < 0) {
            printf("No depth image for frame (%s)\n", labelled_frame->Filename.c_str());
            perror("");
            return false;
        }

        std::cout << "Adding labelled frame " << labelled_frame->Filename << " @ " << labelled_frame->Timestamp.S << " " << labelled_frame->Timestamp.Ns << std::endl;
        file.AddFrame(labelled_frame);
    }
        std::cin.get();

    return true;
}

bool loadScannetPointCloud(const std::string &dirname, SLAMFile &file) {

    const std::string plyname = dirname + "/pointcloud.ply";

    slambench::io::LabelledPointCloudSensor *pcd = new slambench::io::LabelledPointCloudSensor("PointCloud");
    pcd->Description = "Ground truth point cloud";
    pcd->Index = file.Sensors.size();

    file.Sensors.AddSensor(pcd);

   // labelled_sensor->labelMap = readLabels(dirname);
	slambench::io::PlyReader plyreader;
	std::ifstream plyfile_stream(plyname.c_str());
	if(!plyfile_stream.good()) {
		fprintf(stderr, "Could not open PLY file\n");
		return false;
	}
	auto *pointcloud = plyreader.Read(plyfile_stream);
	if(pointcloud == nullptr) {
		fprintf(stderr, "Could not build point cloud\n");
		return false;
	}
	auto rawpointcloud = pointcloud->ToRaw();

	SLAMInMemoryFrame *pcloudframe = new SLAMInMemoryFrame();
	pcloudframe->FrameSensor = file.GetSensor(LabelledPointCloudSensor::kLabelledPointCloudType);
	pcloudframe->Data = malloc(rawpointcloud.size());
	pcloudframe->SetVariableSize(rawpointcloud.size());
	memcpy(pcloudframe->Data, rawpointcloud.data(), rawpointcloud.size());
	file.AddFrame(pcloudframe);


    return true;
}



SLAMFile* ScannetReader::GenerateSLAMFile () {

    std::cout << "=========================" << std::endl;
    std::cout << "Generating SLAM file" << std::endl;
    std::cout << "=========================" << std::endl;

    if (!(rgb || depth)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;
    std::cout << "Input set as " << input << std::endl;

    int noOfFrames = 0;
    if (!analyseScannetFolder(dirname, noOfFrames)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    std::cout << "Found " << noOfFrames << " frames" << std::endl;

    SLAMFile * slamfilep = new SLAMFile();
    SLAMFile & slamfile  = *slamfilep;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    DepthSensor::disparity_params_t disparity_params = {0.001, 0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

    std::cout << "STARTING" << std::endl;

    // Load Depth
    if (depth && !loadScannetDepthData(dirname, slamfile, pose,
                                       intrinsics_depth, distortion_depth,
                                       disparity_params, disparity_type,
                                       noOfFrames)) {
        std::cout << "Error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load RGB
    if (rgb && !loadScannetRGBData(dirname, slamfile, pose, intrinsics_rgb,
                                   distortion_rgb, noOfFrames)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load labelled
    if (labels && !loadScannetLabelledData(dirname, slamfile, noOfFrames)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (pointcloud && !loadScannetPointCloud(dirname, slamfile)) {
        std::cout << "Error while loading Pointcloud information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    std::cout << "DONE" << std::endl;

    return slamfilep;
}

