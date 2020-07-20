/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/sensor/CameraSensor.h"
#include "io/sensor/SensorCollection.h"
#include "io/realsense/RealSense2FrameStream.h"
#include "io/realsense/RealSense2InputInterface.h"

#include <iostream>

using namespace slambench::io;
using namespace slambench::io::realsense;

RealSense2InputInterface::RealSense2InputInterface() : _stream(nullptr), _sensors_ready(false) {
	std::cerr << "OpenNI2 has not been compiled." << std::endl;
	exit(1);
}

RealSense2InputInterface::RealSense2InputInterface(std::string ) : _stream(nullptr), _sensors_ready(false) {
	std::cerr << "OpenNI2 has not been compiled." << std::endl;
	exit(1);
}

FrameStream& RealSense2InputInterface::GetFrames() {
}

SensorCollection& RealSense2InputInterface::GetSensors() {
}

void RealSense2InputInterface::BuildSensors() {
}

void RealSense2InputInterface::BuildStream() {
}
