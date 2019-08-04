#ifndef SPHEROSIM_ERRORMAP
#define SPHEROSIM_ERRORMAP
#include <sphero_error_inject/error_cell.h>
#include <geometry_msgs/Pose.h>

namespace spheroSim {
	class ErrorMap {
		public:
			static ErrorMap* getInstance();
			ErrorCell GetPositionError(geometry_msgs::Pose pose);
			// explicitly delete copy-constructor and copy-assignment
			ErrorMap(const ErrorMap&) = delete;
			ErrorMap& operator=(const ErrorMap&) = delete;
		private:
			ErrorCell* map;
			static ErrorMap* instance;
			ErrorMap();
	};
}

#endif