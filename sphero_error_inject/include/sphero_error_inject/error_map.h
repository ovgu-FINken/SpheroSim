#ifndef SPHEROSIM_ERRORMAP
#define SPHEROSIM_ERRORMAP
#include <sphero_error_inject/error.h>
#include <geometry_msgs/Pose2D.h>

#include <iostream>
#include <fstream>

namespace spheroSim {
	class ErrorMap {
		public:
			static ErrorMap* getInstance();
			Error GetPositionError(geometry_msgs::Pose2D pose);
			// explicitly delete copy-constructor and copy-assignment
			ErrorMap(const ErrorMap&) = delete;
			ErrorMap& operator=(const ErrorMap&) = delete;
		private:
			Error* map;
			static ErrorMap* instance;
			ErrorMap();
	};
}

#endif