/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/deserialisation/SLAMFileDeserialiser.h"
#include "io/deserialisation/SLAMFileHeaderDeserialiser.h"
#include "io/deserialisation/SensorCollectionDeserialiser.h"

#include "io/FrameBuffer.h"
#include "io/FrameBufferSource.h"
#include "io/SLAMFile.h"
#include "io/SLAMFrame.h"

using namespace slambench::io;

bool SLAMFileDeserialiser::Deserialise(SLAMFile &target) {
    return DeserialiseHeader(target) && DeserialiseFrames(target);
}

bool SLAMFileDeserialiser::DeserialiseHeader(SLAMFile &target) {
	SLAMFileHeaderDeserialiser headerDeserialiser(File());
	if(!headerDeserialiser.Deserialise()) {
		return false;
	}
	
	SensorCollectionDeserialiser sensor_deserialiser(File());
    return sensor_deserialiser.Deserialise(target.Sensors);
}

bool SLAMFileDeserialiser::DeserialiseFrames(SLAMFile &target) {
	SLAMFrame *frame;
	
	while(DeserialiseFrame(target, frame)) {
		target.AddFrame(frame);
	}
	
	return true;
}

bool SLAMFileDeserialiser::DeserialiseFrame(SLAMFile &file, SLAMFrame *&target) {
	auto* dsf = new DeserialisedFrame(*GetNextFramebuffer(), File());
	
	if(!Read(&dsf->Timestamp, sizeof(dsf->Timestamp))) {
		delete dsf;
		return false;
	}
	
	uint8_t sensor_index = 0;
	Read(&sensor_index, sizeof(sensor_index));
	dsf->FrameSensor = &file.Sensors.at(sensor_index);
	if(dsf->FrameSensor->IsVariableSize()) {
		uint32_t framesize;
		Read(&framesize, sizeof(framesize));
		dsf->SetVariableSize(framesize);
	}
	
	dsf->SetOffset(Offset());
	Skip(dsf->FrameSensor->GetFrameSize(dsf));
	
	target = dsf;
	
	return true;
}

FrameBuffer *SLAMFileDeserialiser::GetNextFramebuffer() {
	return _fb_source->Next();
}
