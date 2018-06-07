#include "ros/ros.h"
#include "mapping_node/errorInsert.h"
#include "mapping_node/getPositionError.h"


bool insert_error(mapping_node::errorInsert::Request &request, mapping_node::errorInsert::Response &response)
{

}

bool get_point_error (mapping_node::getPositionError::Request &request, mapping_node::getPositionError::Response &response)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "error_mapping");
	ros::NodeHandle n;

	ros::ServiceServer errorInsertServer = n.advertiseService("insert_error", insert_error);
	ros::ServiceServer getErrorServer = n.advertiseService("get_point_error", get_point_error);
	ros::Publisher erroMapPublisher = n.advertise<std::list<ErrorMapGridCell>>("errormap", 1000);
	ros::spin();

	return 0;
}