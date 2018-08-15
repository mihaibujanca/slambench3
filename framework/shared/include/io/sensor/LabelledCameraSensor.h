/*

 Copyright (c) 2018 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_LABELLED_CAMERA_SENSOR_H
#define IO_LABELLED_CAMERA_SENSOR_H

#include <map>

#include "io/sensor/Sensor.h"
#include "io/FrameFormat.h"
#include "io/PixelFormat.h"


namespace slambench {
	namespace io {
		// Sensor for densly labelled frames
		class LabelledCameraSensor : public Sensor {
                private:

		public:

			const static sensor_type_t kLabelType;
			LabelledCameraSensor(const sensor_name_t &name, const sensor_type_t &type = kLabelType);
			
			// Resolution
			uint32_t Width;
			uint32_t Height;
			
			// Color setting
			frameformat::EFrameFormat FrameFormat;
			pixelformat::EPixelFormat PixelFormat;

			size_t GetFrameSize(const SLAMFrame *frame) const override;

                        std::map<int, std::string> labelMap;
		};
	}
}


#endif
