#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include <sphero_error_mapping/error_cell.h>
#include <sphero_error_mapping/error_information.h>

#include <sphero_error_mapping/error_insert.h>
#include <sphero_error_mapping/get_position_error.h>

#include <signal.h>
#include <iostream>
#include <fstream>
#include <vector>

using namespace sphero_error_mapping;

const int MAPSIZE_HORIZONTAL = 800;
const int MAPSIZE_VERTICAL = 600;

std::vector<ErrorCell> map;

/**
 * Callback method to insert a reported error into the shared error map.
 */
bool insert_error(error_insert::Request &request, error_insert::Response &response)
{
    int request_x = request.pose.x;
    int request_y = request.pose.y;
    // find the correct cell in the map to update
    int cellIndex = (MAPSIZE_HORIZONTAL * request_y) + (request_x % MAPSIZE_HORIZONTAL);
    ErrorCell insertCell = map[cellIndex];
    // define the data for the update
    ErrorInformation* report = new ErrorInformation();
    report->age = ros::Time::now().toSec();
    report->linearError = request.linearError;
    report->angularError = request.angularError;
    insertCell.insert_report(report);
    return true;
}

/**
 * Callback method to respond to a request for error information for a given position.
 */
bool get_point_error(get_position_error::Request &request, get_position_error::Response &response)
{
    int request_x = request.pose.x;
    int request_y = request.pose.y;
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

void shutdownHandler() {
	// write the created map out to a file for evaluation
	time_t t = std::time(0);
	long int now = static_cast<long int> (t);
	std::ofstream mapFile;
	mapFile.open("/home/stephan/spheroSim/sphero_error_map_" + std::to_string(now) + ".csv");
	for(int y = 0; y < MAPSIZE_VERTICAL; ++y) {
		for(int x = 0; x < MAPSIZE_HORIZONTAL; ++x) {
			int cellIndex = (y * MAPSIZE_HORIZONTAL) + x;
			ErrorCell cell = map[cellIndex];
    		ErrorInformation *errorInfo = cell.get_error();
			mapFile << std::to_string(errorInfo->linearError) + "|" + std::to_string(errorInfo->angularError) + ";";
		}
		mapFile << "\n";
	}
	mapFile.close();
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
	ros::Rate loop_rate(10);
	// publish
	while(ros::ok()){
		publish_error_map();
		loop_rate.sleep();
	}
	shutdownHandler();

	return 0;
}