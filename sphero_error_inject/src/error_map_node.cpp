#include <sphero_error_inject/error_map.h>
#include <sphero_error_inject/error_cell.h>
#include <sphero_error_inject/error.h>

#include <ros/ros.h>

using namespace sphero_error_inject;

bool getPositionError(error::Request &req, error::Response &res) {
	geometry_msgs::Pose2D pose = req.pose;
	spheroSim::ErrorCell error = spheroSim::ErrorMap::getInstance()->GetPositionError(pose);
	res.linearError = error.linearError;
	res.angularError = error.angularError;
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "error_map_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("get_position_error", getPositionError);
	spheroSim::ErrorMap::getInstance();
	ros::spin();

	return 0;
}