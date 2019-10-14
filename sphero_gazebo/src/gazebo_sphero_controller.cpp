
/*
 * \file  gazebo_sphero_controller.cpp
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


#include <algorithm>
#include <assert.h>

#include <sphero_gazebo/gazebo_sphero_controller.h>

#include <sphero_error_inject/error.h>
#include <sphero_error_mapping/error_insert.h>

#include <ignition/math.hh>
#include <sdf/sdf.hh>

using namespace sphero_error_mapping;
using namespace sphero_error_inject;
using namespace std;
using namespace ignition;

namespace gazebo
{

enum {
    RIGHT,
    LEFT,
};

GazeboSpheroController::GazeboSpheroController(): linear_estimate_(0, 5, 0.5, 0.5), angular_estimate_(0, 5, 0.5, 0.5) {}

// Destructor
GazeboSpheroController::~GazeboSpheroController() {}

// Load the controller
void GazeboSpheroController::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "DiffDrive"));
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "sphero/cmd_vel");
    gazebo_ros_->getParameter<std::string>(odometry_topic_, "odometryTopic", "odom");
    gazebo_ros_->getParameter<std::string>(position_topic_, "positionTopic", "pos");
    gazebo_ros_->getParameter<std::string>(odometry_frame_, "odometryFrame", "/odom_frame");
    gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_footprint");
    gazebo_ros_->getParameterBoolean(publishWheelTF_, "publishWheelTF", false);
    gazebo_ros_->getParameterBoolean(publishWheelJointState_, "publishWheelJointState", false);

    gazebo_ros_->getParameter<double>(wheel_separation_, "wheelSeparation", 0.34);
    gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.1);
    gazebo_ros_->getParameter<double>(wheel_accel, "wheelAcceleration", 0.0);
    gazebo_ros_->getParameter<double>(wheel_torque, "wheelTorque", 0.02);
    gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 10.0);
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource>(odom_source_, "odometrySource", odomOptions, WORLD);

    joints_.resize ( 2 );
    joints_[LEFT] = gazebo_ros_->getJoint(parent, "leftJoint", "left_joint");
    joints_[RIGHT] = gazebo_ros_->getJoint(parent, "rightJoint", "right_joint");
    joints_[LEFT]->SetParam("fmax", 0, wheel_torque);
    joints_[RIGHT]->SetParam("fmax", 0, wheel_torque);

    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN("GazeboSpheroController Plugin (ns = %s) missing <publishTf>, defaults to %d", this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->SimTime();

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    double limit = 0.02;
    uniform_real_distribution<double> unif(0,limit);
    default_random_engine re(time(NULL));
    
    current_linear_error_ = 1 - unif(re);
    current_angular_error_ = 1 - unif(re);
    ROS_INFO("%s: initialized inherent error: %g\t%g", gazebo_ros_->info(), current_linear_error_, current_linear_error_);
    gazebo_ros_->node()->createTimer(ros::Duration(60), boost::bind(&GazeboSpheroController::report_error, this, _1));

    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO("%s: Advertise joint_states!", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboSpheroController::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    reportClient_ = gazebo_ros_->node()->serviceClient<sphero_error_mapping::error_insert>("/insert_error");

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO("%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str());

      position_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Pose2D>(position_topic_, 1);
      ROS_INFO("%s: Advertise position on %s !", gazebo_ros_->info(), position_topic_.c_str());
    }

    errorClient_ = gazebo_ros_->node()->serviceClient<sphero_error_inject::error>("/get_position_error");

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboSpheroController::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboSpheroController::UpdateChild, this ) );

    // initialize prediction
    math::Pose world_pose = parent->GetWorldPose();
    predict_pose_.x = world_pose.pos.x;
    predict_pose_.y = world_pose.pos.y;
    // get the orientation from the simulation
    double theta = world_pose.rot.GetYaw();
    predict_pose_.theta = theta * 180;
}

void GazeboSpheroController::report_error(const ros::TimerEvent& event){
    ROS_INFO("%s: inherent error (%g\t%g)", gazebo_ros_->info(), this->linear_estimate_.X, this->angular_estimate_.X);
}

void GazeboSpheroController::Reset()
{
  last_update_time_ = parent->GetWorld()->SimTime();
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
  joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );
}

void GazeboSpheroController::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < 2; i++ ) {
        physics::JointPtr joint = joints_[i];
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = joint->Position() ;
    }
    joint_state_publisher_.publish(joint_state_);
}

void GazeboSpheroController::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 2; i++ ) {

        string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
        string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

        math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}

// Update the controller
void GazeboSpheroController::UpdateChild()
{

    /* force reset SetMaxForce since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboSpheroController::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    */

    /*
        The real sphero takes velocity commands on a scale of 0 - 255.
        Gazebo (through ros) takes velocity commands in rad/s.

        At maximum linear velocity (255) of 2m/s and small wheel diameter of 2cm 
            maximum angluar velocity: 200 rad/s
        ==> scaling factor is 1/1.275
    */

    for (int i = 0; i < 2; i++) {
      if (fabs(wheel_torque - joints_[i]->GetParam("fmax", 0)) > 1e-6) {
        joints_[i]->SetParam("fmax", 0, wheel_torque);
      }
    }

    if (odom_source_ == ENCODER) {
        UpdateOdometryEncoder();
    }
    common::Time current_time = parent->GetWorld()->SimTime();
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {
        if (this->publish_tf_) {
            publishOdometry();
            publishPosition();
            publishDiff();
            updatePrediction(seconds_since_last_update);
        }
        if (publishWheelTF_) {
            publishWheelTF();
        }
        if (publishWheelJointState_) {
            publishWheelJointState();
        }
        // Update robot in case new velocities have been requested
        getWheelVelocities();
        double current_speed[2];
        current_speed[LEFT] = joints_[LEFT]->GetVelocity(0) * (wheel_diameter_ / 2.0);
        current_speed[RIGHT] = joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0);

        if (wheel_accel == 0 || (fabs(wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01) || (fabs(wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01)) {
            //if max_accel == 0, or target speed is reached
            joints_[LEFT]->SetParam("vel", 0, wheel_speed_[LEFT] / (wheel_diameter_ / 2.0));
            joints_[RIGHT]->SetParam("vel", 0, wheel_speed_[RIGHT] / (wheel_diameter_ / 2.0));
        } else {
            if ( wheel_speed_[LEFT]>=current_speed[LEFT] )
                wheel_speed_instr_[LEFT]+=fmin ( wheel_speed_[LEFT]-current_speed[LEFT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[LEFT]+=fmax ( wheel_speed_[LEFT]-current_speed[LEFT], -wheel_accel * seconds_since_last_update );
            if ( wheel_speed_[RIGHT]>current_speed[RIGHT] )
                wheel_speed_instr_[RIGHT]+=fmin ( wheel_speed_[RIGHT]-current_speed[RIGHT], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[RIGHT]+=fmax ( wheel_speed_[RIGHT]-current_speed[RIGHT], -wheel_accel * seconds_since_last_update );

            joints_[LEFT]->SetParam("vel", 0,wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
            joints_[RIGHT]->SetParam("vel", 0,wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
        }

        last_update_time_+= common::Time(update_period_);
    }
}

// Finalize the controller
void GazeboSpheroController::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboSpheroController::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    double linearFactor = current_linear_error_;
    double angularFactor = current_angular_error_;
    // inject a random error into the next movement instruction
    sphero_error_inject::error errorService;
    errorService.request.pose = pose_;
    if(errorClient_.call(errorService)){
        double linearError = errorService.response.linearError;
        double angularError = errorService.response.angularError;
        linearFactor -= linearError;
        angularFactor -= angularError;
        if(linearFactor < 0 || angularFactor < 0){
            ROS_WARN("%s: negative:\t%g(%g)\t%g(%g)", gazebo_ros_->info(), linearFactor, linearError, angularFactor, angularError);
        }
    } else {
        ROS_ERROR("%s: failed to call error inject.", gazebo_ros_->info());
    }
    double vr = x_ * linearFactor;
    double va = rot_ * angularFactor;
    // hand the movement command over to gazebo
    wheel_speed_[LEFT] = vr;
    wheel_speed_[RIGHT] = va * wheel_separation_ / 2.0;
}

void GazeboSpheroController::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void GazeboSpheroController::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

/**
 * Calculates the odometry for the current step based on the current movement and the last known position.
 * http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
 */
void GazeboSpheroController::UpdateOdometryEncoder()
{
    common::Time current_time = parent->GetWorld()->SimTime();
    double seconds_since_last_update = (current_time - last_odom_update_).Double();
    last_odom_update_ = current_time;
    tf::Vector3 vt;
    tf::Quaternion qt;
    if(x_ == 0) {
        // no movement is currently happening, so odom is just the current position
        vt = tf::Vector3(pose_.x, pose_.y, 0 );
        double theta = pose_.theta + (rot_ * seconds_since_last_update);
        qt.setRPY(0, 0, theta);
    } else if (rot_ == 0) {
        // no turning happens, just movement in a straight line
        double distance = x_ * seconds_since_last_update;
        vt = tf::Vector3(
                (distance * cos(pose_.theta)) + pose_.x,
                (distance * sin(pose_.theta)) + pose_.y,
                0
            );
        qt.setRPY(0, 0, pose_.theta);
    } else {
        Eigen::Vector3d odomTarget = this->calculateCurveMovement(seconds_since_last_update, pose_,  x_, rot_);
        vt = tf::Vector3(odomTarget.x(), odomTarget.y(), 0 );
        qt.setRPY(0, 0, odomTarget.z());
    }

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();
}

Eigen::Vector3d GazeboSpheroController::calculateCurveMovement(double timeInSeconds, geometry_msgs::Pose2D pose, double x, double rot) {
    double currentOrientation = pose.theta;
    // rot_ specifies how long it will take for a full circle (angular velocity in rad/s)
    // x_ specifies how fast the robot travels trough the circle (linear velocity in m/s)
    // x_ / rot_ specifies the radius of the circle
    double fullTurn = 3.14159265358979323846 * 2;
    // specifies how long a full circle will take
    double fullTurnTime = fullTurn / rot;
    double circumference = fullTurnTime * x;
    double radius = circumference / fullTurn; // = x_ / rot_;
    double angle = rot * timeInSeconds;
    // instantanious center of curvature - the point the current curve revolves around
    double iccX = pose.x - (radius * sin(currentOrientation));
    double iccY = pose.y + (radius * cos(currentOrientation));
    Eigen::Matrix3d rotateArountIcc;
    rotateArountIcc <<  cos(angle), -1 * sin(angle), 0,
                        sin(angle), cos(angle), 0,
                        0, 0, 1;
    Eigen::Vector3d translateIccToOrigin(pose.x - iccX, pose.y - iccY, currentOrientation);
    Eigen::Vector3d translateIccBack(iccX, iccY, angle);
    Eigen::Vector3d odomTarget = (rotateArountIcc * translateIccToOrigin) + translateIccBack;
    return odomTarget;
}

void GazeboSpheroController::publishDiff() {

    double distance = sqrt(pow(last_pose_.x - pose_.x, 2) + pow(last_pose_.y - pose_.y, 2));
    double planned_distance = sqrt(pow(last_pose_.x - predict_pose_.pose.pose.position.x, 2) + pow(last_pose_.y - predict_pose_.pose.pose.position.y, 2));
    double relative_distance_diff = 0;
    if(distance != 0 && planned_distance != 0) {
        // ROS_INFO("%s: calculated distance %g and planned distance %g", gazebo_ros_->info(), distance, planned_distance);
        relative_distance_diff = abs((distance / planned_distance) - 1);
    }
    
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(predict_pose_.pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double theta_diff = abs(last_pose_.theta - pose_.theta);
    double planned_theta_diff = abs(last_pose_.theta - yaw);
    double relative_theta_diff = 0;
    if(theta_diff != 0 && planned_theta_diff != 0){
        relative_theta_diff = abs((theta_diff / planned_theta_diff) - 1);
    }
    if(relative_distance_diff == 0 && relative_theta_diff == 0){
        //nothing happened
        return;
    }
    /*
    ROS_INFO("%s: dist %g\t(%g\t%g)", gazebo_ros_->info(), relative_distance_diff, distance, planned_distance);
    ROS_INFO("%s: angl %g\t(%g\t%g)", gazebo_ros_->info(), relative_theta_diff, theta_diff, planned_theta_diff);
    */
    if(relative_distance_diff > 8 || relative_theta_diff > 8){
        // something has gone wrong here
        // ROS_ERROR("%s: relative error too extreme: ( %g | %g ).", gazebo_ros_->info(), relative_distance_diff, relative_theta_diff);
        return;
    }

    // update the inherent error estimate
	this->update_estimate(&linear_estimate_, relative_distance_diff);
	this->update_estimate(&angular_estimate_, relative_theta_diff);
    
    sphero_error_mapping::error_insert mappingService;
    mappingService.request.pose.x = pose_.x;
    mappingService.request.pose.y = pose_.y;
    mappingService.request.linearError = relative_distance_diff;
    mappingService.request.angularError = relative_theta_diff;
    mappingService.request.robotId = '0';
    if(!reportClient_.call(mappingService)){
        ROS_ERROR("%s: failed to call report service.", gazebo_ros_->info());
    }
}

void GazeboSpheroController::update_estimate(KalmanParams *estimate, double measurement) {
	// compute the kalman gain
	double k = estimate->P / ( estimate->P + estimate->R);
	// update the estimation
	estimate->X = estimate->X + (k * (measurement - estimate->X));
	// update the estimation uncertainty
	estimate->P = (1 - k) * estimate->P;
}

void GazeboSpheroController::updatePrediction(double seconds_since_last_update) {
    if(x_ == 0 && rot_ == 0) {
        // no movement, no prediction update
        return;
    } else if(x_ == 0) {
        // no linear movement, so odom is just the current position with new orientation
        predict_pose_.theta = pose_.theta + (rot_ * seconds_since_last_update);
        predict_pose_.x = pose_.x;
        predict_pose_.y = pose_.y;
    } else if (rot_ == 0) {
        // no turning happens, just movement in a straight line
        double distance = x_ * seconds_since_last_update;
        predict_pose_.x = (distance * cos(pose_.theta)) + pose_.x;
        predict_pose_.y = (distance * sin(pose_.theta)) + pose_.y;
        predict_pose_.theta = pose_.theta;
    } else {
        Eigen::Vector3d predictTarget = this->calculateCurveMovement(seconds_since_last_update, pose_,  x_, rot_);
        predict_pose_.x = predictTarget.x();
        predict_pose_.y = predictTarget.y();
        predict_pose_.theta = predictTarget.z();
    }
    // ROS_INFO("%s: current: %g, %g | %g", gazebo_ros_->info(), pose_.x, pose_.y, pose_.theta);
    // ROS_INFO("%s: predict: %g, %g | %g", gazebo_ros_->info(), predict_pose_.x, predict_pose_.y, predict_pose_.theta);
}

void GazeboSpheroController::publishPosition()
{
    // save the previous pose for tracking
    last_pose_ = pose_;
    // get the position from the simulation
    math::Pose3d world_pose = parent->WorldPose();
    pose_.x = world_pose.Pos().X();
    pose_.y = world_pose.Pos().Y();
    // get the orientation from the simulation
    double theta = world_pose.Rot().Yaw();
    pose_.theta = theta * 180;
    position_publisher_.publish(pose_);
}

void GazeboSpheroController::publishOdometry()
{    
    ros::Time current_time = ros::Time::now();

    tf::Quaternion qt;
    tf::Vector3 vt;

    math::Pose3d pose = parent->WorldPose();

    if (odom_source_ == ENCODER) {
        // getting data form encoder integration
        qt = tf::Quaternion(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
        vt = tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
    }
    if ( odom_source_ == WORLD ) {
        qt = tf::Quaternion(0, 0, 0, 1);
        vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();
    }

    // get velocity in /odom frame
    odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();

    // convert velocity to child_frame_id (aka base_footprint)
    math::Vector3d linear = parent->WorldLinearVel();
    double yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
    odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();

    string odom_frame = gazebo_ros_->resolveTF(odometry_frame_);
    string base_footprint_frame = gazebo_ros_->resolveTF(robot_base_frame_);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));

    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboSpheroController)
}

