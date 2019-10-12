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

const int SCALE_FACTOR = 20;
const int MAPSIZE_HORIZONTAL = 3 * SCALE_FACTOR;
const int MAPSIZE_VERTICAL = 4 * SCALE_FACTOR;

std::vector<ErrorCell> map;

/**
 * Callback method to insert a reported error into the shared error map.
 */
bool insert_error(error_insert::Request &request, error_insert::Response &response)
{
	// scale and round the reported position
    int request_x = (int) (request.pose.x * SCALE_FACTOR) + 0.5;
    int request_y = (int) (request.pose.y * SCALE_FACTOR) + 0.5;
    // find the correct cell in the map to update
    int cellIndex = (MAPSIZE_HORIZONTAL * request_y) + (request_x % MAPSIZE_HORIZONTAL);
    ErrorCell insertCell = map[cellIndex];
    // define the data for the update
    ErrorInformation* report = new ErrorInformation();
    report->linearError = request.linearError;
    report->angularError = request.angularError;
    insertCell.insert_report(report);
	map[cellIndex] = insertCell;
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
    response.linearError = errorInfo->linearError;
    response.angularError = errorInfo->angularError;
    return true;
}

void logMap(const ros::TimerEvent& event) {
	// write the created map out to a file for evaluation
	time_t t = std::time(0);
	long int now = static_cast<long int> (t);
	std::ofstream linearFile;
	std::ofstream angularFile;
	linearFile.open("/home/stephan/spheroSim/sphero_error_map_" + std::to_string(now) + "_linear.csv");
	angularFile.open("/home/stephan/spheroSim/sphero_error_map_" + std::to_string(now) + "_angular.csv");
	for(int y = 0; y < MAPSIZE_VERTICAL; ++y) {
		for(int x = 0; x < MAPSIZE_HORIZONTAL; ++x) {
			int cellIndex = (y * MAPSIZE_HORIZONTAL) + x;
			ErrorCell cell = map[cellIndex];
    		ErrorInformation *errorInfo = cell.get_error();
			linearFile << std::to_string(errorInfo->linearError) + "(" + std::to_string(errorInfo->linearCovariance) + ");";
			angularFile << std::to_string(errorInfo->angularError) + "(" + std::to_string(errorInfo->angularCovariance) + ");";
		}
		linearFile << "\n";
		angularFile << "\n";
	}
	linearFile.close();
	angularFile.close();
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
	// log the map every minute
	ros::Timer timer = n.createTimer(ros::Duration(60), logMap);
	ros::spin();

	return 0;
}