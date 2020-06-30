/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <SLAMBenchConfiguration.h>
#include <metrics/ATEMetric.h>
#include <metrics/RPEMetric.h>
#include <outputs/TrajectoryAlignmentMethod.h>
#include <outputs/OutputManagerWriter.h>
#include <metrics/DurationMetric.h>
#include <metrics/PowerMetric.h>
#include <metrics/DepthEstimationMetric.h>
#include <metrics/MemoryMetric.h>
#include <ColumnWriter.h>
#include <SLAMBenchException.h>


std::string default_output_filename;
std::string output_filename;

std::string alignment_technique = "new";
std::string default_alignment_technique = "new";
TypedParameter<std::string> file_output_parameter ("fo", "file-output", "File to write slamfile containing outputs", &output_filename, &default_output_filename);
TypedParameter<std::string> alignment_type_parameter("a",     "alignment-technique",      "Select an alignment technique by name, if not found, \"new alignment\" used (original,new,umeyama).", &alignment_technique, &default_alignment_technique);

int main(int argc, char * argv[])
{

	try {

		auto config = new SLAMBenchConfiguration();

		//***************************************************************************************
		// Start the argument processing
		//***************************************************************************************

		config->addParameter(file_output_parameter);
		config->addParameter(alignment_type_parameter);
		config->GetParameterManager().ReadArgumentsOrQuit(argc, argv);

		//***************************************************************************************
		// At this point the datasets/libraries/sensors are loaded with their arguments set.
		//***************************************************************************************




		//***************************************************************************************
		// We initialise the configuration, means to retrieve groundtruth and set the alignement
		//***************************************************************************************

		config->InitGroundtruth(false);

		// get GT trajectory
		auto gt_traj = config->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);

		slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
		if(alignment_technique == "original") {
			alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
		} else if(alignment_technique == "new") {
			alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
		} else if(alignment_technique == "umeyama") {
            alignment_method = new slambench::outputs::UmeyamaTrajectoryAlignmentMethod();
        } else {
			std::cerr << "Unknown alignment method " << alignment_technique << std::endl;
			return 1;
		}

		//***************************************************************************************
		// We prepare the logging and create the global metrics
		//***************************************************************************************

		config->StartStatistics();
		slambench::ColumnWriter cw (config->GetLogStream(), "\t");
		cw.AddColumn(new slambench::RowNumberColumn());



		auto memory_metric   = std::make_shared<slambench::metrics::MemoryMetric>();
		auto duration_metric = std::make_shared<slambench::metrics::DurationMetric>();
		auto power_metric    = std::make_shared<slambench::metrics::PowerMetric>();

		//***************************************************************************************
		// We init the algos now because we need their output already
		// TODO: if pose and map were by default we could init the algo much later,
		//       thus move memory metric later
		//***************************************************************************************

		config->InitAlgorithms();

		bool have_timestamp = false;

		for(SLAMBenchLibraryHelper *lib : config->GetLoadedLibs()) {
			lib->gt_traj = config->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
			// retrieve the trajectory of the lib
			auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
			if (lib_traj == nullptr) {
				std::cerr << "There is no output trajectory in the library outputs." << std::endl;
				exit(1);
			}

			// Create timestamp column if we don't have one
			if(!have_timestamp) {
				have_timestamp = true;
				cw.AddColumn(new slambench::OutputTimestampColumnInterface(lib_traj));
			}

			if (gt_traj) {
				// Create an aligned trajectory
				auto alignment = new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj), lib_traj, alignment_method);
				alignment->SetActive(true);
				alignment->SetKeepOnlyMostRecent(true);
				auto aligned = new slambench::outputs::AlignedPoseOutput(lib_traj->GetName() + " (Aligned)", alignment, lib_traj);

				// Add ATE metric
				auto ate_metric = std::make_shared<slambench::metrics::ATEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
				if (!ate_metric->GetValueDescription().GetStructureDescription().empty()) {
					lib->GetMetricManager().AddFrameMetric(ate_metric);
					cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*ate_metric, lib->GetMetricManager().GetFramePhase()));
				}

				// Add RPE metric
				auto rpe_metric = std::make_shared<slambench::metrics::RPEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
				lib->GetMetricManager().AddFrameMetric(rpe_metric);
				cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*rpe_metric, lib->GetMetricManager().GetFramePhase()));

			}
//			FIXME: workaround for ground truth
//            auto depth_est_output = lib->GetOutputManager().GetOutput("depth_est");
//            auto depth_est_gt = lib->GetOutputManager().GetOutput("depth_gt");
//            if(!(depth_est_output&&depth_est_gt)) {
//                std::cerr<<"This algorithm does not provide depth estimation"<<std::endl;
//            }
//            else {
//                auto depth_metric = new slambench::metrics::DepthEstimationMetric(depth_est_output,depth_est_gt);
//                lib->GetMetricManager().AddFrameMetric(depth_metric);
//                cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, depth_metric, lib->GetMetricManager().GetFramePhase()));
//            }

			// Add a duration metric
			lib->GetMetricManager().AddFrameMetric(duration_metric);
			lib->GetMetricManager().AddPhaseMetric(duration_metric);
			cw.AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric, lib->GetMetricManager().GetFramePhase()));
			for(auto phase : lib->GetMetricManager().GetPhases()) {
				cw.AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric, phase));
			}

			// Add a memory metric
			lib->GetMetricManager().AddFrameMetric(memory_metric);
			cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*memory_metric, lib->GetMetricManager().GetFramePhase()));

			// Add a power metric if it makes sense
			if (!power_metric->GetValueDescription().GetStructureDescription().empty()) {
				lib->GetMetricManager().AddFrameMetric(power_metric);
				cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*power_metric, lib->GetMetricManager().GetFramePhase()));
			}

			// Add XYZ row from the trajectory
			auto traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
			traj->SetActive(true);
			cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, new slambench::outputs::PoseToXYZOutput(traj)));

		}


		config->AddFrameCallback([&cw]{cw.PrintRow();}); // @suppress("Invalid arguments")
		cw.PrintHeader();

		//***************************************************************************************
		// We run the experiment
		//***************************************************************************************

		SLAMBenchConfiguration::compute_loop_algorithm (config,nullptr,nullptr);

		//***************************************************************************************
		// End of experiment, we output the map
		//***************************************************************************************

		// TODO: Only one output file does not do the job for more than one SLAM systems, output directory maybe ?


		if(!output_filename.empty()) {
		    if(config->GetLoadedLibs().size() > 1) {
                std::cerr << "Can only write outputs to file when there is only one lib loaded" << std::endl;
                return 1;
            }
			// enable all writeable outputs
			auto main_lib = config->GetLoadedLibs().front();

            main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->SetActive(true);
            main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD)->SetActive(true);
            main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_FRAME)->SetActive(true);
			slambench::TimeStamp timestamp = main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->GetMostRecentValue().first;

            main_lib->c_sb_update_outputs(main_lib, &timestamp);

			std::cout << "Writing outputs to " << output_filename << std::endl;
			slambench::outputs::OutputManagerWriter omw;
			omw.Write(main_lib->GetOutputManager(), output_filename);
			std::cout << "Done writing outputs." << std::endl;
		}

		std::cout << "End of program." << std::endl;

		delete config;
		delete alignment_method;


	} catch (const SLAMBenchException& e) {

		std::cout << "An error occurred during the execution." << std::endl;
		std::cout << e.what() << std::endl;

	}

	return 0;
}
