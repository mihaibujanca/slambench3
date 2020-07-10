/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "sb_malloc.h"
#include "metrics/DoubleMetric.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>

using namespace slambench::metrics;

/* Duration Metric */

DoubleMetric::DoubleMetric() : Metric("Double")
{

}

void DoubleMetric::MeasureStart(Phase* )
{
}

void DoubleMetric::MeasureEnd(Phase* )
{
}

Value *DoubleMetric::GetValue(Phase *)
{
	return new values::TypedValue<double>(1234.0);
}


const slambench::values::ValueDescription &DoubleMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::VT_DOUBLE;
	return desc;
}

const std::string &DoubleMetric::GetDescription() const {
	static std::string desc = "Duration of the phase in whole microseconds";
	return desc;
}

