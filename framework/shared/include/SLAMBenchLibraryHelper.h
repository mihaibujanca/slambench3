/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_
#define FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_

#include <Parameters.h>
#include <ParameterComponent.h>
#include <io/InputInterface.h>
#include <metrics/MetricManager.h>
#include <outputs/OutputManager.h>
#include <SLAMBenchUI.h>
#include <utility>
#include <vector>
#include <Eigen/Core>

class SLAMBenchLibraryHelper : public ParameterComponent {

private :
	std::string                        _identifier;
	std::string                        _library_name;
    slambench::metrics::MetricManager  _metric_manager;
    std::ostream&				       _log_stream;
    slambench::io::InputInterface*     _input_interface;
	slambench::outputs::OutputManager  _output_manager;

public:
	bool            (* c_sb_new_slam_configuration)(SLAMBenchLibraryHelper *) ;
    bool            (* c_sb_init_slam_system)(SLAMBenchLibraryHelper * ) ;
    bool            (* c_sb_update_frame) (SLAMBenchLibraryHelper *, slambench::io::SLAMFrame * ) ;
    bool            (* c_sb_process_once) (SLAMBenchLibraryHelper *) ;
    bool            (* c_sb_clean_slam_system)();
    bool            (* c_sb_update_outputs)(SLAMBenchLibraryHelper *, const slambench::TimeStamp *ts);
    bool            (* c_sb_relocalize)(SLAMBenchLibraryHelper * );
	slambench::outputs::BaseOutput* gt_traj;

    SLAMBenchLibraryHelper (const std::string& id,
                            std::string lib,
                            std::ostream& l,
                            slambench::io::InputInterface* i) :
        ParameterComponent(id),
		_identifier(id),
		_library_name(std::move(lib)),
		_log_stream (l),
		_input_interface (i),
		c_sb_new_slam_configuration(nullptr) ,
		c_sb_init_slam_system(nullptr) ,
		c_sb_update_frame(nullptr) ,
		c_sb_process_once(nullptr) ,
		c_sb_clean_slam_system(nullptr) ,
		c_sb_update_outputs(nullptr)
	{}

    inline const std::string& GetIdentifier() const {return _identifier;};
    inline const std::string& GetLibraryName() const {return _library_name;};
    inline std::ostream& GetLogStream() {return _log_stream;};
    inline slambench::metrics::MetricManager &GetMetricManager() { return _metric_manager; }
    inline slambench::outputs::OutputManager &GetOutputManager() { return _output_manager; }
    inline slambench::io::InputInterface *GetInputInterface() {
		if(_input_interface == nullptr) {
			throw std::logic_error("Input interface have not been added to SLAM configuration");
		}
		return _input_interface;
	}

    inline const slambench::io::SensorCollection &GetSensors() {
		return this->GetInputInterface()->GetSensors();
	}

    inline void update_input_interface(slambench::io::InputInterface* interface)
    {
        _input_interface = interface;
    }

};
typedef std::vector<SLAMBenchLibraryHelper*> slam_lib_container_t;
#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_ */
