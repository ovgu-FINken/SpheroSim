#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sstream>
#include <iostream>
#include <thread>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void randomwalk(MoveBaseClient &client) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  while(ros::ok()) {
	goal.target_pose.pose.position.x = 3.5;
	goal.target_pose.pose.position.y = 2;
	client.sendGoal(goal);
	client.waitForResult();
  }
}

/**
 * Entry method to start the randomwalk node.
 */
int main(int argc, char **argv) {
	// initialize the connection to ROS(-master)
	ros::init(argc, argv, "randomwalk_commander");
	// reference the node handle to let ROS start the node
	// Note: ros kills the node when the last handle goes out of scope
	ros::NodeHandle n("~");
	// read the parameter for the number of robots to send around randomly
	int index;
	if(ros::param::get("~index", index)) {
		ROS_INFO("Retrieved index %d.", index);
	} else {
		ROS_ERROR("Could not retrieve index!");
	}
	std::stringstream topicNameStream;
	topicNameStream << "/sphero";
	topicNameStream << index;
	topicNameStream << "/move_base/goal";
	string topicName = topicNameStream.str();
	ROS_INFO("Build topic name %s", topicName.c_str());
	MoveBaseClient client(topicName, true);
	// wait for the move_base servers to come up for all robots
	while(!client.waitForServer(ros::Duration(2.0))){
		ROS_INFO("Wating for move_bases to be ready");
	}

	randomwalk(client);
	ros::spin();
}
