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
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "map";
  double x = 0.0;
  double y = 0.0;
  random_device rd;  //Will be used to obtain a seed for the random number engine
  mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  uniform_real_distribution<double> unif_x(0,3);
  uniform_real_distribution<double> unif_y(0,4);
  while(ros::ok()) {
	x = unif_x(gen);
	y = unif_y(gen);
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation.w = 1;
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
		ROS_INFO("Randomwalk: Retrieved index %d.", index);
	} else {
		ROS_ERROR("Randomwalk: Could not retrieve index!");
	}
	string topicName = "/sphero" + to_string(index) + "/move_base";
	MoveBaseClient client(topicName, true);
	// wait for the move_base servers to come up for all robots
	while(!client.waitForServer(ros::Duration(2.0))){
		ROS_INFO("Wating for move_base to be ready");
	}

	randomwalk(client);
}
