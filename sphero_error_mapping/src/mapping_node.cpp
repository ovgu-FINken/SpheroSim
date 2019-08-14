#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include <sphero_error_mapping/error_cell.h>
#include <sphero_error_mapping/error_information.h>

#include <sphero_error_mapping/error_insert.h>
#include <sphero_error_mapping/get_position_error.h>

#include <vector>

using namespace sphero_error_mapping;

const int MAPSIZE_VERTICAL = 100;
const int MAPSIZE_HORIZONTAL = 100;

std::vector<ErrorCell> map;

/**
 * Callback method to insert a reported error into the shared error map.
 */
bool insert_error(error_insert::Request &request, error_insert::Response &response)
{
    int request_x = request.planned_pose.x;
    int request_y = request.planned_pose.y;
    // find the correct cell in the map to update
    int cellIndex = (MAPSIZE_HORIZONTAL * request_y) + (request_x % MAPSIZE_HORIZONTAL);
    ErrorCell insertCell = map[cellIndex];
    // define the data for the update
    ErrorInformation* report = new ErrorInformation();
    report->age = ros::Time::now().toSec();
    // TODO: handle the difference-transform to put it into the error report.
    insertCell.insert_report(report);
    return true;
}

/**
 * Callback method to respond to a request for error information for a given position.
 */
bool get_point_error(get_position_error::Request &request, get_position_error::Response &response)
{
    int request_x = request.point.x;
    int request_y = request.point.y;
    int cellIndex = (MAPSIZE_HORIZONTAL * request_y) + (request_x % MAPSIZE_HORIZONTAL);
    ErrorCell requestCell = map[cellIndex];
    ErrorInformation *errorInfo = requestCell.get_error();
    response.age = errorInfo->age;
    response.quality = errorInfo->quality;
    response.linearError = errorInfo->linearError;
    response.angularError = errorInfo->angularError;
    return true;
}

/**
 * Publisher method for the complete error map.
 */
void publish_error_map()
{

}

/**
 * Entry method to start the mapping node.
 */
int main(int argc, char **argv)
{
	// initialize the connection to ROS(-master)
	ros::init(argc, argv, "sphero_error_mapping");
	// reference the node handle to let ROS start the node
	// Note: ros kills the node when the last handle goes out of scope
	ros::NodeHandle n;

	// initialize the map with empty error information
	for (int i = 0; i < MAPSIZE_VERTICAL; ++i)
	{
		for (int j = 0; j < MAPSIZE_HORIZONTAL; ++j)
		{
            ErrorCell cell(j, i);
			map.push_back(cell);
		}
	}

	// initialize the services and topics
	ros::ServiceServer errorInsertServer = n.advertiseService("insert_error", insert_error);
	ros::ServiceServer getErrorServer = n.advertiseService("get_point_error", get_point_error);
	ros::spin();

	ros::Rate loop_rate(10);

	// publish
	while(ros::ok()){
		publish_error_map();
		loop_rate.sleep();
	}

	return 0;
}