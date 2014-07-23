/*!=============================================================================
  ==============================================================================

  \ingroup SLros

  \file    SL_ros_publisher.c

  \author  Peter Pastor
  \date    July 2010

  ==============================================================================
  \remarks

      this file contains the functionality to publish SL variables to ROS

  ============================================================================*/
#ifndef SL_ROS_PUBLISHER_H_
#define SL_ROS_PUBLISHER_H_

// first include ROS
#include <ros/ros.h>
#include <rosrt/rosrt.h>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
// #include <sl2ros_msgs/SL2ROS.h>

#include "SL.h"

namespace sl2ros
{

class SL_ros_publisher
{
		
public:
		
	SL_ros_publisher();
	virtual ~SL_ros_publisher();
	
	bool initialize();

	bool publish();

private:

	ros::NodeHandle *node_handle_;
	
	boost::shared_ptr<rosrt::Publisher<sensor_msgs::JointState> > joint_state_publisher_;
	boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > endeffector_publisher_;
	// boost::shared_ptr<rosrt::Publisher<sl2ros_msgs::SL2ROS> > other_msg_publisher_;

};

}

#endif /* SL_ROS_PUBLISHER_H_ */
