#include <sphero_error_inject/error_map.h>

namespace speheroSim {

	ErrorMap* ErrorMap::instance = 0;

	ErrorMap* ErrorMap::getInstance() {
		if(instance == 0) {
			// TODO: pass the path to a csv file
			instance = new ErroMap();
		}
	}

	// empty default constructor
	ErrorMap::ErrorMap() {}

	ErrorMap::ErrorMap(string csvPath, boolean header) {
		// TODO: read the given csv-file
		string* lines = string[];
		// TODO: omit the header, if present
		map = Error[];
		// TODO: initialize the error map from csv lines
	}

	Error ErrorMap::GetPositionError(geometryMsg::Pose2D pose) {
		// TODO: lookup in the map (transform position to array index)
		return map[0];
	}
}