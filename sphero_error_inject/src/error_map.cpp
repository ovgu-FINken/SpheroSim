#include <sphero_error_inject/error_map.h>

namespace speheroSim {
	ErrorMap::ErrorMap() {
		map = ErrorInformation[];
		// TODO: initialize the error map
	}

	ErrorMap::ErrorMap(string csvPath, boolean header) {
		// TODO: read the given csv-file
		string** lines = string[][]
		// TODO: omit the header, if present
		map = ErrorInformation[];
		// TODO: initialize the error map from csv lines
	}

	ErrorInformation ErrorMap::GetPositionError(geometryMsg::Pose2D pose) {
		
	}
}