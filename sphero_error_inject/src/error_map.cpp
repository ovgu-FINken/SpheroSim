#include <sphero_error_inject/error_map.h>

#include <iostream>
#include <fstream>

#include <ros/console.h>

namespace spheroSim {

	const int SCALE_FACTOR = 10;
	const int MAPSIZE_HORIZONTAL = 3 * SCALE_FACTOR;
	const int MAPSIZE_VERTICAL = 4 * SCALE_FACTOR;

	ErrorMap* ErrorMap::instance = 0;

	ErrorMap* ErrorMap::getInstance() {
		if(instance == 0) {
			instance = new ErrorMap();
		}
		return instance;
	}

	// default constructor
	ErrorMap::ErrorMap() {
		// TODO: make this configurable
		const int numSpikes = 10;
		// initialize the map
		ErrorCell map[MAPSIZE_HORIZONTAL * MAPSIZE_VERTICAL];
		// randomly generate positions for "Distotion-Spikes"
	    std::uniform_real_distribution<float> unifX(0, MAPSIZE_HORIZONTAL-1);
	    std::uniform_real_distribution<float> unifY(0, MAPSIZE_VERTICAL-1);
	    std::default_random_engine re;
	    geometry_msgs::Pose2D errorPositions[numSpikes];
	    for (int i = 0; i < numSpikes; ++i)
	    {
		    geometry_msgs::Pose2D errorPos{};
		    errorPos.x = static_cast<int>(unifX(re));
		    errorPos.y = static_cast<int>(unifY(re));
		    errorPositions[i] = errorPos;
	    }
		// write the created map out to a file for later comparison
	    time_t t = std::time(0);
    	long int now = static_cast<long int> (t);
		std::ofstream linearFile;
		std::ofstream angularFile;
		linearFile.open("/home/stephan/spheroSim/sphero_error_init_" + std::to_string(now) + "_linear.csv");
		angularFile.open("/home/stephan/spheroSim/sphero_error_init_" + std::to_string(now) + "_angular.csv");
		// iterate over the map and calculate each cell's error by it's distances to all spikes
		for (int y = 0; y < MAPSIZE_VERTICAL; ++y) {
			for (int x = 0; x < MAPSIZE_HORIZONTAL; ++x) {
				// get the distance to the spikes
				float linearError = 0;
				float angularError = 0;
				for (int s = 0; s < numSpikes; ++s)
				{
					geometry_msgs::Pose2D spike = errorPositions[s];
					double distance = sqrt(pow(spike.x - x, 2) + pow(spike.y - y, 2));
					// error is the inverted distance, so a bigger distance results in a smaller error.
					linearError += 1/distance;
					angularError += 1/distance;
				}
				map[x * y] = ErrorCell(linearError, angularError);
				linearFile << std::to_string(linearError) + ";";
				angularFile << std::to_string(angularError) + ";";
			}
			linearFile << "\n";
			angularFile << "\n";
		}
		linearFile.close();
		angularFile.close();
	}

	ErrorCell ErrorMap::GetPositionError(geometry_msgs::Pose2D pose) {
		// The size of the map, must match the file fed into the navStack
		const int index = (MAPSIZE_HORIZONTAL * (pose.y * SCALE_FACTOR)) + (pose.x * SCALE_FACTOR);
		if(index > (MAPSIZE_HORIZONTAL * MAPSIZE_VERTICAL)) {
			//TODO: Error, out of bounds
			ROS_ERROR("Error injection for requested pose is out of bounds!");
		}
		return map[index];
	}
}