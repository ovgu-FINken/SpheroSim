#ifndef SPHEROSIM_ERRORMAP
#define SPHEROSIM_ERRORMAP
#include <sphero_error_inject/error.h>

namespace spheroSim {
	class ErrorMap {
		public:
			ErrorMap();
			ErrorMap(string csvPath, boolean header);
			Error GetPositionError(geometryMsg::Pose2D pose);
		private:
			Error* map;
	}
}

#endif