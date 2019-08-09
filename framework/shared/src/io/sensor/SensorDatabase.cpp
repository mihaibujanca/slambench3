/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

SensorDatabase *SensorDatabase::singleton_;

SensorDatabase* SensorDatabase::Singleton()
{
	if(singleton_ == nullptr) {
		singleton_ = new SensorDatabase();
	}
	return singleton_;
}

SensorDatabaseEntry& SensorDatabase::Get(const Sensor::sensor_type_t& sensor_name)
{
	return registrations_.at(sensor_name);
}

void SensorDatabase::RegisterSensor(const Sensor::sensor_type_t& sensor_name, const SensorDatabaseEntry& entry)
{
	registrations_.insert({sensor_name, entry});
}

SensorDatabaseRegistration::SensorDatabaseRegistration(const Sensor::sensor_type_t& name, const SensorDatabaseEntry& entry)
{
	SensorDatabase::Singleton()->RegisterSensor(name, entry);
}
