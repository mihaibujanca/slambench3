/*

 Copyright (c) 2018 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SCANNET_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SCANNET_H_

#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include "../../dataset-tools/include/DatasetReader.h"

namespace slambench {

namespace io {

class ScannetReader :  public DatasetReader {

private :

 CameraSensor::intrinsics_t intrinsics_rgb   = { 577.87,
                                                 577.84,
                                                 319.88,
                                                 238.88 };

 DepthSensor::intrinsics_t  intrinsics_depth = { 577.87,
                                                 577.84,
                                                 319.88,
                                                 238.88 };

 CameraSensor::distortion_coefficients_t distortion_rgb   = {0,
                                                             0,
                                                             0,
                                                             0,
                                                             0 };

 DepthSensor::distortion_coefficients_t distortion_depth = { 0,
                                                             0,
                                                             0,
                                                             0,
                                                             0 };

std::map<int, std::string> readLabels(const std::string &dirname);
bool loadScannetLabelledData(const std::string &dirname, SLAMFile &file,
                             const int noOfFrames);

public :
	std::string input;
        bool rgb           = true;
        bool depth         = true;
        bool labels        = true;
        bool pointcloud    = true;

	ScannetReader (std::string name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i",             "input-directory",       "path of the TUM dataset directory",   &this->input, NULL));

        this->addParameter(TypedParameter<bool>("rgb",          "rgb",             "set to true or false to specify if the RGB stream need to be include in the slam file.",   &this->rgb, NULL));
        this->addParameter(TypedParameter<bool>("depth",        "depth",           "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, NULL));
        this->addParameter(TypedParameter<bool>("labels",       "semantic_labels", "set to true or false to specify if the semantic label stream need to be include in the slam file.",   &this->labels, NULL));

	}

	SLAMFile* GenerateSLAMFile () ;


};

}
}



#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SCANNET_H_ */
