#include <sphero_error_inject/error_map.h>

bool getPositionError(sphero_error_inject::Error::Request &req, sphero_error_inject::Error::Response &res) {
	Error error = ErrorMap::getInstance().GetPositionError(req.pose);
	res.linearError = error.linearError;
	res.angularError = error.angularError;
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "error_map_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("get_position_error", getPositionError)
	ros::spin();

	return 0;
}