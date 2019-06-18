#ifndef SPHEROSIM_ERRORMAP
#define SPHEROSIM_ERRORMAP
#include <sphero_error_inject/error.h>

namespace spheroSim {
	class ErrorMap {
		public:
			static ErrorMap* getInstance();
			Error GetPositionError(geometryMsg::Pose2D pose);
			// explicitly delete copy-constructor and copy-assignment
			ErrorMap(const ErrorMap&) = delete;
			ErrorMap& operator=(const ErrorMap&) = delete;
		private:
			Error* map;
			static ErrorMap* instance;
			ErrorMap();
	}
}

#endif