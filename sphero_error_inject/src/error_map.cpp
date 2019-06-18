#include <sphero_error_inject/error_map.h>

namespace speheroSim {

	ErrorMap* ErrorMap::instance = 0;

	ErrorMap* ErrorMap::getInstance() {
		if(instance == 0) {
			instance = new ErroMap();
		}
	}

	// default constructor
	ErrorMap::ErrorMap() {
		const int xSize = 100;
		const int ySize = 100;
		const int numSpikes = 5;
		// initialize the map
		Error map[xSize * ySize];
		// TODO: randomly generate positions for "Distotion-Spikes"
	    std::uniform_real_distribution<int> unifX(0, xSize);
	    std::uniform_real_distribution<int> unifY(0, ySize);
	    std::default_random_engine re;
	    for (int i = 0; i < numSpikes; ++i)
	    {
		    int randX = unifX(re);
		    int randY = unifY(re);
	    }
		// iterate over the map and calculate each cell's error by it's distances to all spikes
		for (int x = 0; x < xSize; ++x) {
			for (int y = 0; y < ySize; ++y) {
				// TODO: get the distance to the spikes
				Error error();
				map[x * y] = error;
			}
		}
	}

	Error ErrorMap::GetPositionError(geometryMsg::Pose2D pose) {
		// TODO: lookup in the map (transform position to array index)
		return map[0];
	}
}