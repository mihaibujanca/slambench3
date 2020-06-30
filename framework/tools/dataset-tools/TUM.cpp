/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "../dataset-tools/include/TUM.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>


#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <fstream>

using namespace slambench::io ;



/*
 *
 * The dataset folder contains :
 * > accelerometer.txt  depth  depth.txt  groundtruth.txt  rgb  rgb.txt
 *
 */

bool analyseTUMFolder(const std::string &dirname) {

    static const std::vector<std::string> requirements = {
            "accelerometer.txt",
            "rgb.txt",
            "rgb",
            "depth.txt",
            "depth",
            "groundtruth.txt"
    };

    try {
        if ( !boost::filesystem::exists( dirname ) ) return false;

        boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
        for ( auto requirement : requirements ) {
            bool seen = false;

            for ( boost::filesystem::directory_iterator itr( dirname ); itr != end_itr; ++itr ) {
                if (requirement == itr->path().filename()) {
                    seen = true;
                }
            }

            if (!seen) {
                std::cout << "File not found: <dataset_dir>/" << requirement << std::endl;
                return false;
            }
        }
    } catch (boost::filesystem::filesystem_error& e)  {
        std::cerr << "I/O Error with directory " << dirname << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}


bool loadTUMDepthData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics,const CameraSensor::distortion_coefficients_t &distortion,  const DepthSensor::disparity_params_t &disparity_params, const DepthSensor::disparity_type_t &disparity_type, const CameraSensor::distortion_type_t &distortion_type) {

    DepthSensor *depth_sensor = new DepthSensor("Depth");
    depth_sensor->Index = 0;
    depth_sensor->Width = 640;
    depth_sensor->Height = 480;
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->Description = "Depth";
    depth_sensor->CopyPose(pose);
    depth_sensor->CopyIntrinsics(intrinsics);
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->DistortionType = distortion_type;
    depth_sensor->CopyRadialTangentialDistortion(distortion);
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Rate = 30.0;

    file.Sensors.AddSensor(depth_sensor);

    std::string line;

    std::ifstream infile(dirname + "/" + "depth.txt");

    boost::smatch match;

    while (std::getline(infile, line)){


        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
            std::string depthfilename = match[3];

            ImageFileFrame *depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor  = depth_sensor;
            depth_frame->Timestamp.S  = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depthfilename ;
            depth_frame->Filename = frame_name.str();

            if(access(depth_frame->Filename.c_str(), F_OK) < 0) {
                printf("No depth image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(depth_frame);



        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}


bool loadTUMRGBData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion, const CameraSensor::distortion_type_t &distortion_type) {

    CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    rgb_sensor->Index = 0;
    rgb_sensor->Width = 640;
    rgb_sensor->Height = 480;
    rgb_sensor->FrameFormat = frameformat::Raster;
    rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
    rgb_sensor->Description = "RGB";
    rgb_sensor->CopyPose(pose);
    rgb_sensor->CopyIntrinsics(intrinsics);
    rgb_sensor->DistortionType = distortion_type;
    rgb_sensor->CopyRadialTangentialDistortion(distortion);
    rgb_sensor->Index =file.Sensors.size();
    rgb_sensor->Rate = 30.0;

    file.Sensors.AddSensor(rgb_sensor);

    std::string line;

    std::ifstream infile(dirname + "/" + "rgb.txt");

    boost::smatch match;

    while (std::getline(infile, line)){


        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
            std::string rgbfilename = match[3];

            ImageFileFrame *rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S  = timestampS;
            rgb_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgbfilename ;
            rgb_frame->Filename = frame_name.str();

            if(access(rgb_frame->Filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(rgb_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }

    }
    return true;
}

bool loadTUMGreyData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion, const CameraSensor::distortion_type_t &distortion_type) {

    CameraSensor *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
    grey_sensor->Index = 0;
    grey_sensor->Width = 640;
    grey_sensor->Height = 480;
    grey_sensor->FrameFormat = frameformat::Raster;
    grey_sensor->PixelFormat = pixelformat::G_I_8;
    grey_sensor->Description = "Grey";

    grey_sensor->CopyPose(pose);
    grey_sensor->CopyIntrinsics(intrinsics);
    grey_sensor->CopyRadialTangentialDistortion(distortion);
    grey_sensor->DistortionType = distortion_type;
    grey_sensor->Index =file.Sensors.size();
    grey_sensor->Rate = 30.0;

    file.Sensors.AddSensor(grey_sensor);

    std::string line;

    std::ifstream infile(dirname + "/" + "rgb.txt");

    boost::smatch match;

    while (std::getline(infile, line)){


        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
            std::string rgbfilename = match[3];

            ImageFileFrame *grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S  = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgbfilename ;
            grey_frame->Filename = frame_name.str();

            if(access(grey_frame->Filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(grey_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }

    }
    return true;
}


bool loadTUMGroundTruthData(const std::string &dirname , SLAMFile &file) {

    GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
    gt_sensor->Index = file.Sensors.size();
    gt_sensor->Description = "GroundTruthSensor";
    file.Sensors.AddSensor(gt_sensor);

    if(!gt_sensor) {
        std::cout << "gt sensor not found..." << std::endl;
        return false;
    } else {
        std::cout << "gt sensor created..." << std::endl;
    }


    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "groundtruth.txt");

    while (std::getline(infile, line)){
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)$"))) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());

            float tx =  std::stof(match[3]);
            float ty =  std::stof(match[4]);
            float tz =  std::stof(match[5]);

            float QX =  std::stof(match[6]);
            float QY =  std::stof(match[7]);
            float QZ =  std::stof(match[8]);
            float QW =  std::stof(match[9]);

            Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW,QX,QY,QZ).toRotationMatrix();
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block(0,0,3,3) = rotationMat;

            pose.block(0,3,3,1) << tx , ty , tz;


            SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
            gt_frame->FrameSensor = gt_sensor;
            gt_frame->Timestamp.S  = timestampS;
            gt_frame->Timestamp.Ns = timestampNS;
            gt_frame->Data = malloc(gt_frame->GetSize());


            memcpy(gt_frame->Data,pose.data(),gt_frame->GetSize());

            file.AddFrame(gt_frame);


        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }


    }
    return true;
}


bool loadTUMAccelerometerData(const std::string &dirname , SLAMFile &file) {

    AccelerometerSensor *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
    accelerometer_sensor->Index = file.Sensors.size();
    accelerometer_sensor->Description = "AccelerometerSensor";
    file.Sensors.AddSensor(accelerometer_sensor);

    if(!accelerometer_sensor) {
        std::cout << "accelerometer_sensor not found..." << std::endl;
        return false;
    }else {
        std::cout << "accelerometer_sensor created..." << std::endl;
    }


    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "accelerometer.txt");

    while (std::getline(infile, line)){

        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)$"))) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
            float ax =  std::stof(match[3]);
            float ay =  std::stof(match[4]);
            float az =  std::stof(match[5]);

            SLAMInMemoryFrame *accelerometer_frame = new SLAMInMemoryFrame();
            accelerometer_frame->FrameSensor = accelerometer_sensor;
            accelerometer_frame->Timestamp.S  = timestampS;
            accelerometer_frame->Timestamp.Ns = timestampNS;
            accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
            ((float*)accelerometer_frame->Data)[0] = ax;
            ((float*)accelerometer_frame->Data)[1] = ay;
            ((float*)accelerometer_frame->Data)[2] = az;

            file.AddFrame(accelerometer_frame);


        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }


    }
    return true;
}






SLAMFile* TUMReader::GenerateSLAMFile () {

    if(!(grey || rgb || depth)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;

    if (!analyseTUMFolder(dirname))	{
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }


    SLAMFile * slamfilep = new SLAMFile();
    SLAMFile & slamfile  = *slamfilep;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    //////  Default are freiburg1

    CameraSensor::intrinsics_t intrinsics_rgb;
    DepthSensor::intrinsics_t intrinsics_depth;

    CameraSensor::distortion_type_t distortion_type;
    CameraSensor::distortion_coefficients_t distortion_rgb;
    DepthSensor::distortion_coefficients_t distortion_depth;

    DepthSensor::disparity_params_t disparity_params;
    DepthSensor::disparity_type_t disparity_type;


    uint32_t kinect = get_sensor_params(disparity_params, disparity_type,
                                        intrinsics_rgb, intrinsics_depth,
                                        distortion_rgb, distortion_depth,
                                        distortion_type);

    if (kinect) {
        std::cout << "using freiburg" << kinect	<< " camera calibration parameters" << std::endl;
    } else {
        std::cout << "using default camera calibration parameters" << std::endl;
        std::cout << "warning: camera calibration might be wrong!" << std::endl;
    }


    /**
     * load Depth
     */

    if(depth && !loadTUMDepthData(dirname, slamfile,pose,intrinsics_depth,distortion_depth,disparity_params,disparity_type,distortion_type)) {
        std::cout << "Error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load Grey
     */

    if(grey && !loadTUMGreyData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb,distortion_type)) {
        std::cout << "Error while loading Grey information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load RGB
     */

    if(rgb && !loadTUMRGBData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb,distortion_type)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load GT
     */
    if(gt && !loadTUMGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading gt information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load Accelerometer: This one failed
     */
    if(accelerometer && !loadTUMAccelerometerData(dirname, slamfile)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    return slamfilep;
}


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/TUM.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "include/utils/sensor_builder.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <Eigen/Eigen>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <fstream>
#include <iostream>

using namespace slambench::io;

constexpr DepthSensor::disparity_params_t TUMReader::disparity_params;
constexpr DepthSensor::disparity_type_t TUMReader::disparity_type;

constexpr CameraSensor::intrinsics_t TUMReader::fr1_intrinsics_rgb;
constexpr DepthSensor::intrinsics_t  TUMReader::fr1_intrinsics_depth;
constexpr CameraSensor::intrinsics_t TUMReader::fr2_intrinsics_rgb;
constexpr DepthSensor::intrinsics_t  TUMReader::fr2_intrinsics_depth;
constexpr CameraSensor::intrinsics_t TUMReader::fr3_intrinsics_rgb;
constexpr DepthSensor::intrinsics_t  TUMReader::fr3_intrinsics_depth;
constexpr CameraSensor::intrinsics_t TUMReader::ethl_intrinsics_rgb;
constexpr DepthSensor::intrinsics_t  TUMReader::ethl_intrinsics_depth;

constexpr CameraSensor::distortion_coefficients_t TUMReader::fr1_distortion_rgb;
constexpr CameraSensor::distortion_coefficients_t TUMReader::fr2_distortion_rgb;
constexpr CameraSensor::distortion_coefficients_t TUMReader::fr3_distortion_rgb;
constexpr CameraSensor::distortion_coefficients_t TUMReader::ethl_distortion_rgb;
constexpr DepthSensor::distortion_coefficients_t  TUMReader::fr1_distortion_depth;
constexpr DepthSensor::distortion_coefficients_t  TUMReader::fr2_distortion_depth;
constexpr DepthSensor::distortion_coefficients_t  TUMReader::fr3_distortion_depth;
constexpr DepthSensor::distortion_coefficients_t  TUMReader::ethl_distortion_depth;

bool loadTUMDepthData(const std::string &dirname,
                      SLAMFile &file,
                      DepthSensor* depth_sensor) {

    std::string line;

    std::ifstream infile(dirname + "/" + "depth.txt");

    boost::smatch match;

    boost::regex comment = boost::regex(RegexPattern::comment);

    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;
    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& fn = RegexPattern::filename;

    // format: timestamp filename
    std::string expr = start
                       + ts  + ws       // timestamp
                       + fn + end;      // filename

    boost::regex depth_line = boost::regex(expr);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, depth_line)) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string depth_filename = match[3];

            auto depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor = depth_sensor;
            depth_frame->Timestamp.S = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depth_filename;
            depth_frame->Filename = frame_name.str();

            if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
                printf("No depth image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(depth_frame);

        } else {
            std::cout << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadTUMRGBData(const std::string &dirname,
                    SLAMFile &file,
                    CameraSensor* rgb_sensor) {

    std::string line;

    std::ifstream infile(dirname + "/" + "rgb.txt");

    boost::smatch match;

    boost::regex comment = boost::regex(RegexPattern::comment);

    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;
    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& fn = RegexPattern::filename;

    // format: timestamp filename
    std::string expr = start
                       + ts  + ws       // timestamp
                       + fn + end;      // filename

    boost::regex rgb_line = boost::regex(expr);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, rgb_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string rgb_filename = match[3];

            auto rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S = timestampS;
            rgb_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgb_filename;
            rgb_frame->Filename = frame_name.str();

            if (access(rgb_frame->Filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(rgb_frame);

        } else {
            std::cout << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadTUMGreyData(const std::string &dirname, SLAMFile &file, CameraSensor* grey_sensor) {

    std::string line;

    std::ifstream infile(dirname + "/" + "rgb.txt");

    boost::smatch match;

    boost::regex comment = boost::regex(RegexPattern::comment);

    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;
    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& fn = RegexPattern::filename;

    // format: timestamp filename
    std::string expr = start
                       + ts  + ws       // timestamp
                       + fn + end;      // filename

    boost::regex rgb_line = boost::regex(expr);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, rgb_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string rgb_filename = match[3];

            auto grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgb_filename;
            grey_frame->Filename = frame_name.str();

            if (access(grey_frame->Filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(grey_frame);

        } else {
            std::cout << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadTUMGroundTruthData(const std::string &dirname, SLAMFile &file, GroundTruthSensor* gt_sensor) {

    std::ifstream infile(dirname + "/" + "groundtruth.txt");

    std::string line;
    boost::smatch match;

    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& dec = RegexPattern::decimal;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // format: timestamp tx ty tz qx qy qz qw
    std::string expr = start
                       + ts  + ws       // timestamp
                       + dec + ws       // tx
                       + dec + ws       // ty
                       + dec + ws       // tz
                       + dec + ws       // qx
                       + dec + ws       // qy
                       + dec + ws       // qz
                       + dec + end;     // qw

    boost::regex groundtruth_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, groundtruth_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            float tx = std::stof(match[3]);
            float ty = std::stof(match[4]);
            float tz = std::stof(match[5]);

            float QX = std::stof(match[6]);
            float QY = std::stof(match[7]);
            float QZ = std::stof(match[8]);
            float QW = std::stof(match[9]);

            Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block(0, 0, 3, 3) = rotationMat;

            pose.block(0, 3, 3, 1) << tx, ty, tz;

            auto gt_frame = new SLAMInMemoryFrame();
            gt_frame->FrameSensor = gt_sensor;
            gt_frame->Timestamp.S = timestampS;
            gt_frame->Timestamp.Ns = timestampNS;
            gt_frame->Data = malloc(gt_frame->GetSize());

            memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

            file.AddFrame(gt_frame);

        } else {
            std::cout << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadTUMAccelerometerData(const std::string &dirname, SLAMFile &file, AccelerometerSensor* accelerometer_sensor) {

    std::ifstream infile(dirname + "/" + "accelerometer.txt");

    std::string line;
    boost::smatch match;

    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& dec = RegexPattern::decimal;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // format: timestamp ax ay az
    std::string expr = start
                       + ts  + ws       // timestamp
                       + dec + ws       // ax
                       + dec + ws       // ay
                       + dec + end;     // az

    boost::regex accelerometer_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, accelerometer_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            float ax = std::stof(match[3]);
            float ay = std::stof(match[4]);
            float az = std::stof(match[5]);

            auto accelerometer_frame = new SLAMInMemoryFrame();
            accelerometer_frame->FrameSensor = accelerometer_sensor;
            accelerometer_frame->Timestamp.S = timestampS;
            accelerometer_frame->Timestamp.Ns = timestampNS;
            accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
            ((float *)accelerometer_frame->Data)[0] = ax;
            ((float *)accelerometer_frame->Data)[1] = ay;
            ((float *)accelerometer_frame->Data)[2] = az;

            file.AddFrame(accelerometer_frame);

        } else {
            std::cout << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

SLAMFile *TUMReader::GenerateSLAMFile() {

    if (!(grey || rgb || depth)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }
    bool is_ethl = false;
    std::string dirname = input;

    const std::vector<std::string> requirements = {"accelerometer.txt",
                                                   "rgb.txt",
                                                   "rgb",
                                                   "depth.txt",
                                                   "depth",
                                                   "groundtruth.txt"};

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfile_ptr = new SLAMFile();
    auto &slamfile = *slamfile_ptr;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    //////  Default are freiburg1

    CameraSensor::intrinsics_t intrinsics_rgb;
    DepthSensor::intrinsics_t intrinsics_depth;

    CameraSensor::distortion_coefficients_t distortion_rgb;
    DepthSensor::distortion_coefficients_t distortion_depth;

    if (dirname.find("freiburg1") != std::string::npos) {
        std::cout << "This dataset is assumed to be using freiburg1." << std::endl;

        for (int i = 0; i < 4; i++) {
            intrinsics_rgb[i]   = fr1_intrinsics_rgb[i];
            intrinsics_depth[i] = fr1_intrinsics_depth[i];
            distortion_rgb[i]   = fr1_distortion_rgb[i];
            distortion_depth[i] = fr1_distortion_depth[i];
        }
        distortion_rgb[4]   = fr1_distortion_rgb[4];
        distortion_depth[4] = fr1_distortion_depth[4];

    } else if (dirname.find("freiburg2") != std::string::npos) {
        std::cout << "This dataset is assumed to be using freiburg2." << std::endl;
        for (int i = 0; i < 4; i++) {
            intrinsics_rgb[i]   = fr2_intrinsics_rgb[i];
            intrinsics_depth[i] = fr2_intrinsics_depth[i];
            distortion_rgb[i]   = fr2_distortion_rgb[i];
            distortion_depth[i] = fr2_distortion_depth[i];
        }
        distortion_rgb[4]   = fr2_distortion_rgb[4];
        distortion_depth[4] = fr2_distortion_depth[4];


    }
    else if (dirname.find("freiburg3") != std::string::npos) {
        std::cout << "This dataset is assumed to be using freiburg3." << std::endl;
        for (int i = 0; i < 4; i++) {
            intrinsics_rgb[i]   = fr3_intrinsics_rgb[i];
            intrinsics_depth[i] = fr3_intrinsics_depth[i];
            distortion_rgb[i]   = fr3_distortion_rgb[i];
            distortion_depth[i] = fr3_distortion_depth[i];
        }

    } else if (dirname.find("ethl") != std::string::npos)
    {
        is_ethl = true;
        for (int i = 0; i < 4; i++) {
            intrinsics_rgb[i]   = ethl_intrinsics_rgb[i];
            intrinsics_depth[i] = ethl_intrinsics_depth[i];
            distortion_rgb[i]   = ethl_distortion_rgb[i];
            distortion_depth[i] = ethl_distortion_depth[i];
        }
    }
    else  {
        std::cout << "Camera calibration might be wrong !." << std::endl;
        delete slamfile_ptr;
        return nullptr;
    }


    // load GT
    if (gt) {
        auto gt_sensor = GTSensorBuilder()
                .name("GroundTruth")
                .description("GroundTruthSensor")
                .build();

        gt_sensor->Index = slamfile.Sensors.size();
        slamfile.Sensors.AddSensor(gt_sensor);

        if(!loadTUMGroundTruthData(dirname, slamfile, gt_sensor)) {
            std::cerr << "Error while loading gt information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load Accelerometer
    if (accelerometer) {
        auto accelerometer_sensor = AccSensorBuilder()
                .name("Accelerometer")
                .description("AccelerometerSensor")
                .build();

        accelerometer_sensor->Index = slamfile.Sensors.size();
        slamfile.Sensors.AddSensor(accelerometer_sensor);

        if (!loadTUMAccelerometerData(dirname, slamfile, accelerometer_sensor)) {
            std::cout << "Error while loading Accelerometer information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load Depth
    if (depth) {
        auto depth_sensor = DepthSensorBuilder()
                .name("Depth")
                .rate(30.0)
                .size(640, 480)
                .pose(pose)
                .intrinsics(intrinsics_depth)
                .distortion(CameraSensor::distortion_type_t::RadialTangential, distortion_depth)
                .disparity(disparity_type, disparity_params)
                .build();

        depth_sensor->Index = slamfile.Sensors.size();
        slamfile.Sensors.AddSensor(depth_sensor);
        if (is_ethl)
            dirname = input + "/depth";
        if (!loadTUMDepthData(dirname, slamfile, depth_sensor)) {
            std::cout << "Error while loading depth information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load Grey
    if (grey) {
        auto grey_sensor = GreySensorBuilder()
                .name("Grey")
                .rate(30.0)
                .size(640, 480)
                .pose(pose)
                .intrinsics(intrinsics_rgb)
                .distortion(CameraSensor::distortion_type_t::RadialTangential, distortion_rgb)
                .build();

        grey_sensor->Index = slamfile.Sensors.size();
        slamfile.Sensors.AddSensor(grey_sensor);
        if (is_ethl)
            dirname = input + "/rgb";
        if (!loadTUMGreyData(dirname, slamfile, grey_sensor)) {
            std::cerr << "Error while loading Grey information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load RGB
    if (rgb) {
        auto rgb_sensor = RGBSensorBuilder()
                .name("RGB")
                .rate(30.0)
                .size(640, 480)
                .pose(pose)
                .intrinsics(intrinsics_rgb)
                .distortion(CameraSensor::distortion_type_t::RadialTangential, distortion_rgb)
                .build();

        rgb_sensor->Index = slamfile.Sensors.size();
        slamfile.Sensors.AddSensor(rgb_sensor);

        if (!loadTUMRGBData(dirname, slamfile, rgb_sensor)) {
            std::cerr << "Error while loading RGB information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    return slamfile_ptr;
}




