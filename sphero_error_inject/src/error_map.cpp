#include <sphero_error_inject/error_map.h>

namespace speheroSim {

	ErrorMap* ErrorMap::instance = 0;

	ErrorMap* ErrorMap::getInstance() {
		if(instance == 0) {
			instance = new ErrorMap();
		}
		return &instance;
	}

	// default constructor
	ErrorMap::ErrorMap() {
		// The size of the map, must match the file fed into the navStack
		const int xSize = 100;
		const int ySize = 100;
		// TODO: make this configurable
		const int numSpikes = 5;
		// initialize the map
		Error map[xSize * ySize];
		// randomly generate positions for "Distotion-Spikes"
	    std::uniform_real_distribution<int> unifX(0, xSize);
	    std::uniform_real_distribution<int> unifY(0, ySize);
	    std::default_random_engine re;
	    geometryMsg::Pose2D errorPositions[numSpikes];
	    for (int i = 0; i < numSpikes; ++i)
	    {
		    geometryMsg::Pose2D errorPos();
		    errorPos.x = unifX(re);
		    errorPos.y = unifY(re);
		    errorPositions[i] = errorPos;
	    }
		// write the created map out to a file for later comparison
	    time_t t = std::time(0);
    	long int now = static_cast<long int> (t);
		std::ofstram mapFile;
		mapFile.open("/home/stephan/sphero_map.csv");
		// iterate over the map and calculate each cell's error by it's distances to all spikes
		for (int y = 0; y < ySize; ++y) {
			for (int x = 0; x < xSize; ++x) {
				// get the distance to the spikes
				float linearError = 0;
				float angularError = 0;
				for (int s = 0; s < numSpikes; ++s)
				{
					geometryMsg::Pose2D spike = errorPositions[s];
					double distance = sqrt(pow(spike.x - x) + pow(spike.y - y));
					// error is the inverted distance, so a bigger distance results in a smaller error.
					linearError += 1/distance;
					angularError += 1/distance;
				}
				map[x * y] = Error(linearError, angularError);
				mapFile << linearError.to_string() + "|" + angularError.to_string() + ";";
			}
			mapFile << "\n";
		}
		mapFile.close();
	}

	Error ErrorMap::GetPositionError(geometryMsg::Pose2D pose) {
		// TODO: lookup in the map (transform position to array index)
		return map[0];
	}
}