/*!=============================================================================
  ==============================================================================

  \ingroup SLros


  \file    SL_ros_communicator.c

  \author  Peter Pastor
  \date    July 2010

  ==============================================================================
  \remarks

      this file contains the functionality to publish SL variables to ROS

  ============================================================================*/

// ROS includes
#include <ros/spinner.h>

// local includes
#include "SL_ros_communicator.h"

// SL includes
#include "SL.h"

namespace sl2ros
{

SL_ros_communicator::SL_ros_communicator()
{
}

SL_ros_communicator::~SL_ros_communicator()
{
}

bool SL_ros_communicator::initialize()
{

    ROS_INFO("Initializing SL_ros_communicator.");

		// Note: this will be done once and probably goes somewhere else...
//    int argc = 1;
//    char name[] = "SL2ROS_Publisher";
//    char* argv[1];
//    argv[0] = name;
//    ros::init(argc, argv, "SL2ROS_Publisher");
    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    node_handle_.reset(new ros::NodeHandle());

    // pre-allocate msgs:
    joint_state_msg_.reset(new sensor_msgs::JointState());
    pose_stamped_msg_.reset(new geometry_msgs::PoseStamped());

    joint_state_msg_->name.resize(n_dofs);
    joint_state_msg_->position.resize(n_dofs);
    joint_state_msg_->velocity.resize(n_dofs);
    joint_state_msg_->effort.resize(n_dofs);
	
    for(int i=1; i<=n_dofs; ++i)
    {
        joint_state_msg_->name[i-1] = joint_names[i];
    }

    ROS_INFO("Publishing %i joint states.", n_dofs);
		nrt_joint_state_publisher_ = node_handle_->advertise<sensor_msgs::JointState>("JointStates", 1000);

    ROS_INFO("Publishing end-effector position."); // I am not sure whether this is needed.
		nrt_endeffector_publisher_ = node_handle_->advertise<geometry_msgs::PoseStamped>("EndeffectorPose", 1000);

    ros::spinOnce();

    ROS_INFO("Done.");
    return true;
}

bool SL_ros_communicator::publish()
{
    // publish joint states in NON REAL-TIME
    joint_state_msg_->header.seq = 1;
    joint_state_msg_->header.stamp = ros::Time::now();
    for(int i=1; i<=n_dofs; ++i) {
            joint_state_msg_->position[i-1] = joint_state[i].th;
            joint_state_msg_->velocity[i-1] = joint_state[i].thd;
            joint_state_msg_->effort[i-1] = joint_state[i].u;
    }
    nrt_joint_state_publisher_.publish(joint_state_msg_);

    // publish endeffector pose in NON REAL-TIME
    pose_stamped_msg_->header.seq = 1;
    pose_stamped_msg_->header.stamp = ros::Time::now();
    int i=1;
    pose_stamped_msg_->pose.position.x = cart_state[i].x[_X_];
    pose_stamped_msg_->pose.position.y = cart_state[i].x[_Y_];
    pose_stamped_msg_->pose.position.z = cart_state[i].x[_Z_];
    nrt_endeffector_publisher_.publish(pose_stamped_msg_);

//    ros::spinOnce();
    return true;
}

}
