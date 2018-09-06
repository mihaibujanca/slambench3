/*

 Copyright (c) 2018 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/SLAMFrame.h"
#include "io/sensor/LabelledPointCloudSensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

const Sensor::sensor_type_t LabelledPointCloudSensor::kLabelledPointCloudType = "LabelledPointCloud";

LabelledPointCloudSensor::LabelledPointCloudSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kLabelledPointCloudType)
{

}

size_t LabelledPointCloudSensor::GetFrameSize(const SLAMFrame *frame) const {
	return frame->GetVariableSize();
}

class LPCSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		// nothing to do
		(void)serialiser;
		(void)sensor;
		return true;
	}
};

class LPCDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != LabelledPointCloudSensor::kLabelledPointCloudType) {
			return false;
		}

		*s = new LabelledPointCloudSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration rgb_reg(LabelledPointCloudSensor::kLabelledPointCloudType, slambench::io::SensorDatabaseEntry(new LPCSerialiser(), new LPCDeserialiser(), true, true));
