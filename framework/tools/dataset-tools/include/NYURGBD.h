/*

 Copyright (c) 2018 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NYURGBD_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NYURGBD_H_

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

class NYURGBDReader :  public DatasetReader {

private :

 CameraSensor::intrinsics_t intrinsics_rgb   = { 5.1885790117450188e+02,
                                                 5.1946961112127485e+02,
                                                 3.2558244941119034e+02,
                                                 2.5373616633400465e+02 };
 DepthSensor::intrinsics_t  intrinsics_depth = { 5.8262448167737955e+02,
                                                 5.8269103270988637e+02,
                                                 3.1304475870804731e+02,
                                                 2.3844389626620386e+02 };

 CameraSensor::distortion_coefficients_t distortion_rgb   = { 2.0796615318809061e-01,
                                                             -5.8613825163911781e-01,
                                                              7.2231363135888329e-04,
                                                              1.0479627195765181e-03,
                                                              4.9856986684705107e-01 };
 DepthSensor::distortion_coefficients_t distortion_depth = { -9.9897236553084481e-02,
                                                              3.9065324602765344e-01,
                                                              1.9290592870229277e-03,
                                                             -1.9422022475975055e-03,
                                                             -5.1031725053400578e-01 };


public :
	std::string input;
	bool grey          = true;
        bool rgb           = true;
        bool depth         = true;
        bool depth_filled  = true;
        bool labels        = true;
        bool accelerometer = false;

	NYURGBDReader (std::string name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i",             "input-directory",       "path of the TUM dataset directory",   &this->input, NULL));

        this->addParameter(TypedParameter<bool>("rgb",          "rgb",             "set to true or false to specify if the RGB stream need to be include in the slam file.",   &this->rgb, NULL));
        this->addParameter(TypedParameter<bool>("depth",        "depth",           "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, NULL));
        this->addParameter(TypedParameter<bool>("depth_filled", "depth_filled",    "set to true or false to specify if the FILLED DEPTH stream need to be include in the slam file.",   &this->depth_filled, NULL));
        this->addParameter(TypedParameter<bool>("labels",       "semantic_labels", "set to true or false to specify if the semantic label stream need to be include in the slam file.",   &this->labels, NULL));
        this->addParameter(TypedParameter<bool>("acc",          "accelerometer",   "set to true or false to specify if the ACCELEROMETER stream need to be include in the slam file.",   &this->accelerometer, NULL));

	}

	SLAMFile* GenerateSLAMFile () ;


};

}
}



#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NYURGBD_H_ */
