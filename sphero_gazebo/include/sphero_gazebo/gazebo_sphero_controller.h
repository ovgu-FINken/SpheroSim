/*
 * \file  gazebo_sphero_controller.h
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot. The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * A modification from the original Differential drive of Gazebo
 * To make the Sphero simulated robot move using any keyboard teleop
 * \author   Ricardo Tellez <rtellez@theconstructsim.com>
 * \date 11th of Aug 2016
 *
 * $ Id: 08/11/2016 20:05:40 PM ouroboros $
 */

#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

#include <cmath>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

//Eigen
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include "sphero_error_mapping/error_insert.h"

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace sphero_error_mapping;

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboSpheroController : public ModelPlugin {

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
    public:
      GazeboSpheroController();
      ~GazeboSpheroController();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      void getWheelVelocities();
      void publishDiff();
      void updatePrediction(double seconds_since_last_update);
      void publishOdometry();
      void publishPosition();
      void publishWheelTF(); /// publishes the wheel tf's
      void publishWheelJointState();
      void QueueThread();
      void UpdateOdometryEncoder();
      double getErrorFactor(double limit);
      Eigen::Vector3d calculateCurveMovement(double timeInSeconds, geometry_msgs::Pose2D pose, double x, double rot);

      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      double wheel_separation_;
      double wheel_diameter_;
      double wheel_torque;
      double wheel_speed_[2];
      double wheel_accel;
      double wheel_speed_instr_[2];

      double current_linear_error_;
      double current_angular_error_;

      std::vector<physics::JointPtr> joints_;

      // ROS STUFF
      ros::Publisher odometry_publisher_;
      ros::Publisher position_publisher_;
      ros::Publisher joint_state_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      ros::ServiceClient errorClient_;
      ros::ServiceClient reportClient_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      sensor_msgs::JointState joint_state_;
      nav_msgs::Odometry odom_;
      geometry_msgs::Pose2D pose_;
      geometry_msgs::Pose2D predict_pose_;
      geometry_msgs::Pose2D last_pose_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string position_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      bool publish_tf_;
      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;

      double x_;
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      OdomSource odom_source_;
      geometry_msgs::Pose2D pose_encoder_;
      common::Time last_odom_update_;

      // Flags
      bool publishWheelTF_;
      bool publishWheelJointState_;

  };

}

#endif

