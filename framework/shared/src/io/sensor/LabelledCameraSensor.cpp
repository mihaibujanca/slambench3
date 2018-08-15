
#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"
#include "io/sensor/LabelledCameraSensor.h"
#include "io/sensor/SensorDatabase.h"
#include "io/SLAMFrame.h"

#include <cassert>
#include <iostream>

using namespace slambench::io;

const Sensor::sensor_type_t LabelledCameraSensor::kLabelType = "Labels";

LabelledCameraSensor::LabelledCameraSensor(const Sensor::sensor_name_t &name,
					   const Sensor::sensor_type_t &sensor_type) :
		Sensor(name, sensor_type),
		Width(0),
		Height(0),
		FrameFormat(slambench::io::frameformat::UNKNOWN),
		PixelFormat(slambench::io::pixelformat::UNKNOWN)
{
}

size_t LabelledCameraSensor::GetFrameSize(const SLAMFrame *frame) const {
	(void)frame;
	return frame->GetVariableSize();
}

class LabelledCameraSensorSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		LabelledCameraSensor *sensor = (LabelledCameraSensor*)s;

		serialiser->Write(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		serialiser->Write(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		serialiser->Write(&sensor->Width, sizeof(sensor->Width));
		serialiser->Write(&sensor->Height, sizeof(sensor->Height));

                size_t map_size = sensor->labelMap.size();
                serialiser->Write(&map_size, sizeof(map_size));

                for (const auto &entry : sensor->labelMap) {
                    serialiser->Write(&entry.first, sizeof(entry.first));
                    size_t str_len = entry.second.size() + 1;
                    serialiser->Write(&str_len, sizeof(str_len));
                    serialiser->Write(entry.second.c_str(), str_len);
                }

		return true;
	}
};

class LabelledCameraSensorDeserialiser : public SensorDeserialiser {
	
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if (type == LabelledCameraSensor::kLabelType) {
			*s = new LabelledCameraSensor(sensor_name, type);
			return true;
		} else {
			return false;
		}
	}

	bool DeserialiseSensorSpecific(Deserialiser* deserialiser, Sensor* s) override {
		LabelledCameraSensor *sensor = (LabelledCameraSensor*)s;
		
		assert(sensor->GetType() == LabelledCameraSensor::kLabelType);
		
		deserialiser->Read(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		deserialiser->Read(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		deserialiser->Read(&sensor->Width, sizeof(sensor->Width));
		deserialiser->Read(&sensor->Height, sizeof(sensor->Height));
		
                size_t map_size = 0;
                deserialiser->Read(&map_size, sizeof(map_size));

                for (size_t index = 0; index < map_size; index++) {
                    int key = 0;
                    deserialiser->Read(&key, sizeof(key));

                    size_t str_len = 0;
                    deserialiser->Read(&str_len, sizeof(str_len));

                    std::vector<char> val(str_len);
                    deserialiser->Read(val.data(), str_len);

                    sensor->labelMap.emplace(key, val.data());
                }

		return true;
	}
};

static slambench::io::SensorDatabaseRegistration labelled_camera_reg(LabelledCameraSensor::kLabelType, slambench::io::SensorDatabaseEntry(new LabelledCameraSensorSerialiser(), new LabelledCameraSensorDeserialiser(), true, true));
