#include <ros/ros.h>

#include <mavconn.h>			// MAVCONN includes (this includes MAVLINK and LCM)
#include <glib.h>

#include "mavlinkros.h"			// Helper functions to convert a MAVLINK message in to a MAVLINK-ROS message and vice versa
#include "mavlink_attitude_ros.h"	// Helper function to convert a MAVLINK attitude message to sensor_msgs/Imu message

ros::Publisher mavlink_pub;
ros::Publisher attitude_pub;
ros::Publisher vicon_pub;
ros::Publisher COMMAND_pub;


std::string lcmurl = "udpm://"; ///< host name for UDP server
bool verbose;

lcm_t *lcm;

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	if (verbose)
		ROS_INFO("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

	/**
	 * Serialize the Mavlink-ROS-message
	 */
	lcm_mavlink_ros::Mavlink rosmavlink_msg;
	createROSFromMavlink(msg,&rosmavlink_msg);

	/**
	 * Mark the ROS-Message as coming from LCM so that it will not be sent back to LCM
	 */
	rosmavlink_msg.fromlcm = true;

	/**
	 * Send the received MAVLink message to ROS (topic: mavlink, see main())
	 */
	mavlink_pub.publish(rosmavlink_msg);

	switch(msg->msgid)
	{
	case MAVLINK_MSG_ID_COMMAND_LONG:
		{
                        lcm_mavlink_ros::COMMAND COMMAND_msg;
                        convertMavlinkCOMMANDToROS(msg, COMMAND_msg);
                        COMMAND_pub.publish(COMMAND_msg);

			if (verbose)
				ROS_INFO("Published COMMAND message (sys:%d|comp:%d):\n", msg->sysid, msg->compid);
		}
		break;
	case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
		{
                        geometry_msgs::PoseStamped vicon_msg;
                        convertMavlinkVicon_Position_EstimateToROS(msg, vicon_msg);
                        vicon_pub.publish(vicon_msg);

			if (verbose)
				ROS_INFO("Published Vicon message (sys:%d|comp:%d):\n", msg->sysid, msg->compid);
		}

		break;
	case MAVLINK_MSG_ID_ATTITUDE:
		{
                        sensor_msgs::Imu imu_msg;
                        convertMavlinkAttitudeToROS(msg, imu_msg);
                        attitude_pub.publish(imu_msg);

			if (verbose)
				ROS_INFO("Published Imu message (sys:%d|comp:%d):\n", msg->sysid, msg->compid);
		}
		break;
	}
}



void* lcm_wait(void* lcm_ptr) {
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1) {
		if (verbose) printf("LCM Handle...");
		lcm_handle(lcm);
	}
	return NULL;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lcmtoros");

	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "lcmurl", 'l', 0, G_OPTION_ARG_STRING, &lcmurl, "LCM Url to connect to", "udpm://" },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0 }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- translate MAVLink messages from LCM to ROS (Topic: mavlink)");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	ros::NodeHandle mavlink_nh;
        mavlink_pub = mavlink_nh.advertise<lcm_mavlink_ros::Mavlink> ("/fromMAVLINK", 1);

	ros::NodeHandle attitude_nh;
        attitude_pub = attitude_nh.advertise<sensor_msgs::Imu>("/fromMAVLINK/Imu", 1);


	ros::NodeHandle vicon_nh;
        vicon_pub = vicon_nh.advertise<geometry_msgs::PoseStamped>("/fromMAVLINK/Vicon", 1);

	ros::NodeHandle COMMAND_nh;
        COMMAND_pub = vicon_nh.advertise<lcm_mavlink_ros::COMMAND>("/fromMAVLINK/COMMAND", 1);

	/**
	 * Connect to LCM Channel and register for MAVLink messages
	 */
	lcm_t* lcm = lcm_create(lcmurl.c_str());
	if (!lcm) {
		return 1;
	}

	mavconn_mavlink_msg_container_t_subscription_t * comm_sub = mavconn_mavlink_msg_container_t_subscribe(
			lcm, "MAVLINK", &mavlink_handler, NULL);

	while (ros::ok())
		lcm_handle(lcm);

	/*
	// Initialize LCM receiver thread
	GThread* lcm_thread;
	GError* err;

	if (!g_thread_supported()) {
		g_thread_init( NULL);
		// Only initialize g thread if not already done
	}

	if ((lcm_thread = g_thread_create((GThreadFunc) lcm_wait, (void *) lcm,
			TRUE, &err)) == NULL) {
		printf("Thread create failed: %s!!\n", err->message);
		g_error_free ( err);
	}*/

	//ros::Rate loop_rate(1);

	/**
	 * Now just wait until the process is terminated...
	 */
	//while (ros::ok()) {
	//	loop_rate.sleep();
	//}

	mavconn_mavlink_msg_container_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
//	if (verbose) printf("Trying GThread Join");
//	g_thread_join(lcm_thread);

	return 0;
}
