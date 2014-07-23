/*!=============================================================================
  ==============================================================================

  \ingroup SLros

  \file    SL_ros_communication.c

  \author  Peter Pastor
  \date    July 2010

  ==============================================================================
  \remarks

      this file contains the functionality to publish SL variables to ROS

  ============================================================================*/
#ifndef SL_ROS_PUBLISHER_H_
#define SL_ROS_PUBLISHER_H_

// boost includes
#include <boost/shared_ptr.hpp>

// ROS includes
#include <ros/ros.h>

// ROS msgs includes
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace sl2ros
{

class SL_ros_communicator
{
		
public:

	SL_ros_communicator();
	virtual ~SL_ros_communicator();
	
	bool initialize();
	bool publish();

private:

        boost::shared_ptr<ros::NodeHandle> node_handle_;
	ros::Publisher nrt_joint_state_publisher_;
	ros::Publisher nrt_endeffector_publisher_;
        boost::shared_ptr<sensor_msgs::JointState> joint_state_msg_;
        boost::shared_ptr<geometry_msgs::PoseStamped> pose_stamped_msg_;

};

}

#endif /* SL_ROS_PUBLISHER_H_ */
