#include "mavlink.h"
#include "lcm_mavlink_ros/Mavlink.h"	// ROS message definition
#include "lcm_mavlink_ros/COMMAND.h"

/**
 * Takes a ROS-Mavlink-message (mavlink_ros_msg) and converts it into a Mavlink-Message (msg)
 */
static inline void createMavlinkFromROS(const lcm_mavlink_ros::Mavlink::ConstPtr& mavlink_ros_msg, mavlink_message_t *msg)
{
	msg->msgid = mavlink_ros_msg->msgid;

	//Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
	//copy(mavlink_ros_msg->payload.begin(), mavlink_ros_msg->payload.end(), msg->payload64);
	//uint8_t buf[MAVLINK_MAX_MSG_LEN];
	//uint16_t len = mavlink_msg_to_send_buffer(&buf, msg);

	//mavlink_finalize_message(msg, mavlink_ros_msg->sysid, mavlink_ros_msg->compid, mavlink_ros_msg->len);
}

/**
 * Takes a Mavlink-message (mavlink_msg) and converts it into a ROS-Mavlink-Message (mavlink_ros_msg)
 */
static inline void createROSFromMavlink(const mavlink_message_t* mavlink_msg, lcm_mavlink_ros::Mavlink* mavlink_ros_msg)
{
	mavlink_ros_msg->len = mavlink_msg->len;
	mavlink_ros_msg->seq = mavlink_msg->seq;
	mavlink_ros_msg->sysid = mavlink_msg->sysid;
	mavlink_ros_msg->compid = mavlink_msg->compid;
	mavlink_ros_msg->msgid = mavlink_msg->msgid;
	mavlink_ros_msg->fromlcm = false;

	for (int i = 0; i < mavlink_msg->len; i++)
	{
		(mavlink_ros_msg->payload).push_back(mavlink_msg->payload64[i]);
	}
}
