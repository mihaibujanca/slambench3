/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <SLAMBenchAPI.h>
#include <SLAMBenchConfiguration.h>
#include <SLAMBenchUI_Pangolin.h>
#include <thread>

#include <io/InputInterface.h>
#include <io/FrameBufferSource.h>
#include <io/SLAMFrame.h>
#include <io/sensor/Sensor.h>
#include <SLAMBenchException.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "ColumnWriter.h"

#include <metrics/SemanticPixelMetric.h>
#include <metrics/ConfusionMatrixMetric.h>
#include <metrics/DurationMetric.h>
#include <metrics/DoubleMetric.h>

std::string alignment_technique = "original";
std::string default_alignment_technique = "original";
TypedParameter<std::string> alignment_type_parameter("a",     "alignment-technique",      "Select an alignment technique by name, if not found, default used (default,new).", &alignment_technique, &default_alignment_technique);

std::string dictionary_filepath;
std::string default_dictionary_filepath = "";
TypedParameter<std::string> dictionary_filepath_parameter("d", "dictionary", "Filepath to a map between the dataset semantic classes and the algorithm semantic classes",
                                                          &dictionary_filepath, &default_dictionary_filepath);


std::map<int, int> read_dictionary(const std::string &dictionary_filename) {
	std::map<int, int> dictionary;

	std::ifstream dictionary_reader(dictionary_filename);
	if (!dictionary_reader.good()) {
		const std::string message = "Could not load dictonary file: " + dictionary_filename;
		std::cerr << message << std::endl;
		throw message;
	}

	std::string dump;
	std::getline(dictionary_reader, dump);
	std::getline(dictionary_reader, dump);

	int gt_class, algo_class;
	while (dictionary_reader >> gt_class >> algo_class)
		dictionary[gt_class] = algo_class;

	return dictionary;
}


void run_pangolin(bool *stay_on, SLAMBenchConfiguration *config);
static SLAMBenchUI * volatile ui = nullptr;

int main(int argc, char * argv[])
{

	try {

		SLAMBenchConfiguration * config = new SLAMBenchConfiguration();

		//***************************************************************************************
		// Start the argument processing
		//***************************************************************************************


		config->getParameters().push_back(&alignment_type_parameter);
		config->getParameters().push_back(&dictionary_filepath_parameter);
		config->GetParameterManager().ReadArgumentsOrQuit(argc, argv, config);
		//***************************************************************************************
		// At this point the datasets/libraries/sensors are loaded with their arguments set.
		//***************************************************************************************


		//***************************************************************************************
		// We initialise the configuration, means to retrieve groundtruth and set the alignement
		//***************************************************************************************

		config->InitGroundtruth();

		// get GT trajectory
		auto gt_trajectory = config->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);

		slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
		if(alignment_technique == "original") {
			alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
		} else if(alignment_technique == "new") {
			alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
		} else {
			std::cerr << "Unknown alignment method " << alignment_technique << std::endl;
			return 1;
		}

		//***************************************************************************************
		// We prepare the logging and create the global metrics
		//***************************************************************************************

		config->start_statistics();
		slambench::ColumnWriter cw (config->get_log_stream(), "\t");
		cw.AddColumn(new slambench::RowNumberColumn());

		bool have_timestamp = false;

		//***************************************************************************************
		// We init the algos now because we need their output already
		// TODO: if pose and map were by default we could init the algo much later,
		//       thus move memory metric later
		//***************************************************************************************

		config->InitAlgorithms();

		// If a ground truth is available, produce an aligned trajectory for each algorithm
		if(gt_trajectory) {
			for(auto lib : config->GetLoadedLibs()) {
				// trajectory
				slambench::outputs::BaseOutput *trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);

				// produce an alignment
				auto alignment = new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_trajectory), trajectory, alignment_method);
				alignment->SetActive(true);
				alignment->SetKeepOnlyMostRecent(true);

				auto aligned = new slambench::outputs::AlignedPoseOutput(trajectory->GetName() + " (Aligned)", alignment, trajectory);
				lib->GetOutputManager().RegisterOutput(aligned);

				// try and align a pointcloud
				slambench::outputs::BaseOutput *pointcloud = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD);
				slambench::outputs::BaseOutput *colouredpointcloud = lib->GetOutputManager().GetMainOutput(slambench::values::VT_COLOUREDPOINTCLOUD);
				if(pointcloud != nullptr) {
					auto pc_aligned = new slambench::outputs::AlignedPointCloudOutput(pointcloud->GetName() + "(Aligned)", alignment, pointcloud);
					lib->GetOutputManager().RegisterOutput(pc_aligned);
				}

				if(colouredpointcloud != nullptr) {
					auto pc_aligned = new slambench::outputs::AlignedColouredPointCloudOutput(colouredpointcloud->GetName() + "(Aligned)", alignment, colouredpointcloud);
					lib->GetOutputManager().RegisterOutput(pc_aligned);
				}

				// Create timestamp column if we don't have one
				if(!have_timestamp) {
					have_timestamp = true;
					cw.AddColumn(new slambench::OutputTimestampColumnInterface(trajectory));
				}
			}
		}

                for(auto lib : config->GetLoadedLibs()) {
                        // Add a memory metric
                        auto semantic_projection_output = lib->GetOutputManager().GetOutput("Semantic Projection");
                        if (semantic_projection_output == nullptr) {
                            std::cerr << "The library does not set any semantic projection output" << std::endl;
                        } else {
                            auto &gt_manager = config->GetGroundTruth();
                            auto gt_segmentation = gt_manager.GetMainOutput(slambench::values::VT_LABELLEDFRAME);

                            if (gt_segmentation) {
                                const std::map<int, int> class_dictionary = read_dictionary(dictionary_filepath);

                                auto semantic_metric = new slambench::metrics::SemanticPixelMetric(semantic_projection_output, gt_segmentation, class_dictionary);
                                lib->GetMetricManager().AddFrameMetric(semantic_metric);
                                cw.AddColumn(new slambench::ValueLibColumnInterface(lib, semantic_metric, lib->GetMetricManager().GetFramePhase()));

                                auto confusion_matrix_metric = new slambench::metrics::ConfusionMatrixMetric(semantic_projection_output, gt_segmentation, class_dictionary);
                                lib->GetMetricManager().AddFrameMetric(confusion_matrix_metric);
                                cw.AddColumn(new slambench::CollectionValueLibColumnInterface(lib, confusion_matrix_metric, lib->GetMetricManager().GetFramePhase()));
                            }
                        }
		}

		//***************************************************************************************
		// We Start the GUI
		//***************************************************************************************


		bool stay_on = true;
		std::thread pangolin_thread(run_pangolin, &stay_on, config);
		while(ui == nullptr) ; // spin until UI is initialised
		// We Start the Experiment
		//***************************************************************************************

		config->AddFrameCallback([&cw]{cw.PrintRow();});
		cw.PrintHeader();

		SLAMBenchConfiguration::compute_loop_algorithm( config, &stay_on, ui);

		//***************************************************************************************
		// We End and wait until the GUI stop
		//***************************************************************************************

		stay_on = false;
		pangolin_thread.join();

	} catch (const SLAMBenchException& e) {

		std::cout << "An error occurred during the execution." << std::endl;
		std::cout << e.what() << std::endl;

	}

	return 0;
}

void run_pangolin(bool *stay_on, SLAMBenchConfiguration *config) {
	std::cerr << "Creation of GUI interface." << std::endl;
	SLAMBenchUI_Pangolin * ui_pangolin = new SLAMBenchUI_Pangolin();
	ui_pangolin->AddOutputManager("Ground Truth", &config->GetGroundTruth());
	for(auto lib : config->GetLoadedLibs()) {
		ui_pangolin->AddOutputManager(lib->getName(), &lib->GetOutputManager());
	}
	ui_pangolin->InitialiseOutputs();

	ui = dynamic_cast<SLAMBenchUI*> (ui_pangolin);

	// Provide the current camera position to the GUI
	// FIXME : temporary values (Toky)
	float fx = 520.9000244141;
	float fy = 521.0000000000;
	float cx = 324.5999755859;
	float cy = 249.2000122070;
	ui->update_camera(480,640, fx, fy , cx , cy );

	std::cerr << "Start rendering loop." << std::endl;
	while( !pangolin::ShouldQuit()) {

		usleep(40000);
		if (!ui->process ()) {
			std::cerr << "Rendering problem." << std::endl;
			exit(1);
		}
	}
	std::cout << "Stop Pangolin..." << std::endl;
	pangolin::Quit();
	std::cout << "GUI closed ... " << std::endl;


	//***************************************************************************************
	// CLOSE COMPUTE THREADS
	//***************************************************************************************

	*stay_on = false;

	std::cout << "Wait for compute thread..." << std::endl;

	//	compute_thread.join();



	//***************************************************************************************
	// CLOSE LIBRARIES
	//***************************************************************************************

	std::cout << "End of program." << std::endl;

}
