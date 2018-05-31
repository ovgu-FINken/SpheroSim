import base_local_planner.h

def computeVelocityCommands 	(cmf_vel):
	return False


def initialize ( name, tf, costmap_ros ):
	if not initialized_:
		costmap_ros_= costmap_ros
		costmap_= costmap_ros_.getCostmap()
	# initialize other planner parameters
           	ros::NodeHandle private_nh("~/" + name);
          	private_nh.param("step_size", step_size_, costmap_->getResolution());
          	private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
         	world_model_ = new base_local_planner::CostmapModel(*costmap_);  
   	      	initialized_ = true;
       
       	else:
         	ROS_WARN("This planner has already been initialized... doing nothing")

def setPlan(plan):
	if not initialized_:
     		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
       		return false;

