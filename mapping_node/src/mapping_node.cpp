#include "ros/ros.h"
#include "mapping_node/errorInsert.h"
#include "mapping_node/getPositionError.h"
#include <vector>

using namespace mapping_node;

const int MAPSIZE_VERTICAL = 100;
const int MAPSIZE_HORIZONTAL = 100;

std::vector<ErrorCell> map;

///
/// Callback method to insert a reported error into the shared error map.
///
bool insert_error(errorInsert::Request &request, errorInsert::Response &response)
{
	int request_x = request.planned_pose.linear.x;
	int request_y = request.planned_pose.linear.y;
	int cellIndex = (MAPSIZE_HORIZONTAL * request.y) + (request_x % MAPSIZE_HORIZONTAL);
	ErrorCell *insertCeĺl = map[cellIndex];
	insertCeĺl.insert_report(request);
}

///
/// Callback method to respond to a request for error information for a given position.
///
bool get_point_error (getPositionError::Request &request, getPositionError::Response &response)
{

}

///
/// Publisher method for the complete error map.
///
void publish_error_map()
{

}

///
/// Entry method to start the mapping node.
///
int main(int argc, char **argv)
{
	// initialize the connection to ROS(-master)
	ros::init(argc, argv, "error_mapping");
	// reference the node handle to let ROS start the node
	// Note: ros kills the node when the last handle goes out of scope
	ros::NodeHandle n;

	// initialize the map with empty error information
	for (int i = 0; i < MAPSIZE_VERTICAL; ++i)
	{
		for (int j = 0; j < MAPSIZE_HORIZONTAL; ++j)
		{
			map.push_back(new ErrorCell(j, i));
		}
	}

	// initialize the services and topics
	ros::ServiceServer errorInsertServer = n.advertiseService("insert_error", insert_error);
	ros::ServiceServer getErrorServer = n.advertiseService("get_point_error", get_point_error);
	ros::Publisher erroMapPublisher = n.advertise<std::list<errorMapGridCell>>("errormap", 1000);
	ros::spin();

	ros::Rate loop_rate(10);

	// publish
	while(ros::ok()){
		publish_error_map();
		loop_rate.sleep();
	}

	return 0;
}