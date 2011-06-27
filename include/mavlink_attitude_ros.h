#include "mavlink.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

static inline void convertMavlinkAttitudeToROS(const mavlink_message_t *msg, sensor_msgs::Imu &imu_msg)
{
	mavlink_attitude_t amsg;
	mavlink_msg_attitude_decode(msg, &amsg);

	btQuaternion quat;
	quat.setRPY(amsg.roll, amsg.pitch, amsg.yaw);
	imu_msg.orientation.x = quat.x();
	imu_msg.orientation.y = quat.y();
	imu_msg.orientation.z = quat.z();
	imu_msg.orientation.w = quat.w();
	imu_msg.angular_velocity.x = amsg.rollspeed;
	imu_msg.angular_velocity.y = amsg.pitchspeed;
	imu_msg.angular_velocity.z = amsg.yawspeed;
}

static inline void convertMavlinkVicon_Position_EstimateToROS(const mavlink_message_t *msg, geometry_msgs::PoseStamped &vicon_msg)
{
	mavlink_vicon_position_estimate_t vmsg;
	mavlink_msg_vicon_position_estimate_decode(msg, &vmsg);

	btQuaternion quat;
	quat.setRPY(vmsg.roll, vmsg.pitch, vmsg.yaw);
	vicon_msg.pose.orientation.x = quat.x();
	vicon_msg.pose.orientation.y = quat.y();
	vicon_msg.pose.orientation.z = quat.z();
	vicon_msg.pose.orientation.w = quat.w();
	vicon_msg.pose.position.x=vmsg.x;
	vicon_msg.pose.position.y=vmsg.y;
	vicon_msg.pose.position.z=vmsg.z;
}
