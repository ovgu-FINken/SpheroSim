#include <sphero_error_inject/error_map.h>

#include <iostream>
#include <fstream>

#include <ros/console.h>

namespace spheroSim {

	ErrorMap* ErrorMap::instance = 0;

	ErrorMap* ErrorMap::getInstance() {
		if(instance == 0) {
			instance = new ErrorMap();
		}
		return instance;
	}

	// default constructor
	ErrorMap::ErrorMap() {
		// The size of the map, must match the file fed into the navStack
		const int xSize = 100;
		const int ySize = 100;
		// TODO: make this configurable
		const int numSpikes = 5;
		// initialize the map
		ErrorCell map[xSize * ySize];
		ROS_INFO("starting to generate error spikes...");
		// randomly generate positions for "Distotion-Spikes"
	    std::uniform_real_distribution<float> unifX(0, xSize);
	    std::uniform_real_distribution<float> unifY(0, ySize);
	    std::default_random_engine re;
	    geometry_msgs::Pose errorPositions[numSpikes];
	    for (int i = 0; i < numSpikes; ++i)
	    {
		    geometry_msgs::Pose errorPos{};
		    errorPos.position.x = static_cast<int>(unifX(re));
		    errorPos.position.y = static_cast<int>(unifY(re));
		    errorPositions[i] = errorPos;
	    }
		// write the created map out to a file for later comparison
	    time_t t = std::time(0);
    	long int now = static_cast<long int> (t);
		std::ofstream mapFile;
		mapFile.open("/home/stephan/sphero_map.csv");
		ROS_INFO("starting to generate map...");
		// iterate over the map and calculate each cell's error by it's distances to all spikes
		for (int y = 0; y < ySize; ++y) {
			for (int x = 0; x < xSize; ++x) {
				// get the distance to the spikes
				float linearError = 0;
				float angularError = 0;
				for (int s = 0; s < numSpikes; ++s)
				{
					geometry_msgs::Pose spike = errorPositions[s];
					double distance = sqrt(pow(spike.position.x - x, 2) + pow(spike.position.y - y, 2));
					// error is the inverted distance, so a bigger distance results in a smaller error.
					linearError += 1/distance;
					angularError += 1/distance;
				}
				map[x * y] = ErrorCell(linearError, angularError);
				mapFile << std::to_string(linearError) + "|" + std::to_string(angularError) + ";";
			}
			mapFile << "\n";
		}
		ROS_INFO("writing map file...");
		mapFile.close();
		ROS_INFO("Done writing file.");
	}

	ErrorCell ErrorMap::GetPositionError(geometry_msgs::Pose pose) {
		// TODO: lookup in the map (transform position to array index)
		return map[0];
	}
}