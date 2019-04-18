#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sstream>
#include <iostream>
#include <thread>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void randomwalk(MoveBaseClient *client) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  while(ros::ok()) {
	goal.target_pose.pose.position.x = 3.5;
	goal.target_pose.pose.position.y = 2;
	client->sendGoal(goal);
	client->waitForResult();
  }
}

void sendCommands(std::vector<MoveBaseClient*> &clients) {
	std::thread *threads = new std::thread[clients.size()];
	for (int i = 0; i < clients.size(); ++i)
	{
		threads[i] = std::thread(randomwalk, clients[i]);
	}
	// std::for_each(clients.begin(), clients.end(), [](MoveBaseClient client){ std::thread(randomwalk, client); });
}

/**
 * Entry method to start the randomwalk node.
 */
int main(int argc, char **argv) {
	// initialize the connection to ROS(-master)
	ros::init(argc, argv, "randomwalk_commander");
	ROS_INFO("--------------------------HELLO WORLD!");
	// reference the node handle to let ROS start the node
	// Note: ros kills the node when the last handle goes out of scope
	ros::NodeHandle n;

	// read the parameter for the number of robots to send around randomly
	int numRobots;
	n.getParam("numRobots", numRobots);
	ROS_INFO("Creating %d clients...", numRobots);
	std::vector<MoveBaseClient*> clients;
	for (int i = 0; i < numRobots; ++i)
	{
		std::stringstream topicNameStream;
		topicNameStream << "/sphero";
		topicNameStream << i+1; 			// robot-topics start at 1
		topicNameStream << "/move_base/goal";
		string topicName = topicNameStream.str();
		MoveBaseClient client(topicName, true);
		clients.push_back(&client);
		ROS_INFO("Client %d created.", i);
	}
	ROS_INFO("All clients created. Wating...");
	// wait for the move_base servers to come up for all robots
	while(!clients[0]->waitForServer(ros::Duration(2.0))){
		ROS_INFO("Wating for move_bases to be ready");
	}

	sendCommands(clients);

	ros::spin();
}
