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
		const int xSize = 4000;
		const int ySize = 3000;
		// TODO: make this configurable
		const int numSpikes = 10;
		// initialize the map
		ErrorCell map[xSize * ySize];
		ROS_INFO("starting to generate error spikes...");
		// randomly generate positions for "Distotion-Spikes"
	    std::uniform_real_distribution<float> unifX(0, xSize-1);
	    std::uniform_real_distribution<float> unifY(0, ySize-1);
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
					geometry_msgs::Pose2D spike = errorPositions[s];
					double distance = sqrt(pow(spike.x - x, 2) + pow(spike.y - y, 2));
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
		// The size of the map, must match the file fed into the navStack
		const int xSize = 4000;
		const int ySize = 3000;
		const int index = (xSize * pose.position.y) + pose.position.x;
		if(index > (xSize * ySize)) {
			//TODO: Error, out of bounds
			ROS_ERROR("Error injection for requested pose is out of bounds!")
		}
		return map[index];
	}
}