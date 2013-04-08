/*
 * mav_status.cpp
 *
 *  Created on: Jan 27, 2012
 *      Author: acmarkus
 */

#include "mav_status.h"

MavStatus::MavStatus():last_update_fcu_(ros::Time(0))
{
  ros::NodeHandle nh;

  sub_fcu_status_ = nh.subscribe("fcu/status", 100, &MavStatus::fcuStatusCallback, this);
  sub_fcu_rc_ = nh.subscribe("fcu/rcdata", 100, &MavStatus::fcuRcCallback, this);
  sub_ssdk_debug_ = nh.subscribe("fcu/debug", 100, &MavStatus::ssdkCallback, this);

  pub_status_ = nh.advertise<mav_status::Status>("mav_status", 10);

  ns_ = ros::this_node::getNamespace();

  // workaround for namespace bug #3617, https://code.ros.org/trac/ros/ticket/3617
  while(ns_.at(0) == '/'){
    ns_.erase(0, 1);
    if(ns_.empty())
      break;
  }
  ns_ = "/" + ns_;

}

void MavStatus::fcuStatusCallback(const asctec_hl_comm::mav_statusConstPtr & msg)
{
  status_msg_.header.stamp = msg->header.stamp;
  last_update_fcu_ = ros::Time::now(); // use current time, otherwise log replay won't work

  status_msg_.gps_lock = msg->gps_status == "GPS fix";
  status_msg_.gps_num_satellites = msg->gps_num_satellites;

  status_msg_.hl_interface_enabled = msg->serial_interface_enabled;
}

void MavStatus::fcuRcCallback(const asctec_hl_comm::mav_rcdataConstPtr & msg)
{
  status_msg_.header.stamp = msg->header.stamp;

  uint16_t si = msg->channel[4]; // serial interface
  uint16_t fm = msg->channel[5]; // flight mode

  if (si > 4000)
  {
    if (fm > 2000 && fm < 2100)
      status_msg_.mav_controller_mode = mav_status::Status::MAV_CONTROLLER_HL_HEIGHT;
    else if (fm > 4000)
      status_msg_.mav_controller_mode = mav_status::Status::MAV_CONTROLLER_HL_POS;
    else
      status_msg_.mav_controller_mode = mav_status::Status::MAV_CONTROLLER_MANUAL;
  }
  else
  {
    if (fm > 2000 && fm < 2100)
      status_msg_.mav_controller_mode = mav_status::Status::MAV_CONTROLLER_LL_HEIGHT;
    else if (fm > 4000)
      status_msg_.mav_controller_mode = mav_status::Status::MAV_CONTROLLER_LL_GPS;
    else
      status_msg_.mav_controller_mode = mav_status::Status::MAV_CONTROLLER_MANUAL;
  }
}


void MavStatus::ssdkCallback(const asctec_hl_comm::DoubleArrayStampedConstPtr & msg)
{
  status_msg_.header.stamp = msg->header.stamp;

  status_msg_.current_setpoint_pos.x = msg->data[24];
  status_msg_.current_setpoint_pos.y = msg->data[25];
  status_msg_.current_setpoint_pos.z = msg->data[26];
  //TODO: get yaw setpoint
}

void MavStatus::sendStatus()
{
  static unsigned int seq = 0;

  status_msg_.header.seq = seq;
  status_msg_.header.frame_id = ns_;

  status_msg_.fcu_alive = watchdog(last_update_fcu_);

  pub_status_.publish(status_msg_);
}

bool MavStatus::watchdog(const ros::Time & time)
{
  return (ros::Time::now() - time) < ros::Duration(WATCHDOG_TIMEOUT);
}


int main(int argc, char** argv){

  ros::init(argc, argv, "status");

  ros::NodeHandle pnh("~");
  double rate;
  pnh.param("rate", rate, 2.0);

  ros::Rate r(rate);
  MavStatus ms;

  while(ros::ok()){
    ros::spinOnce();
    ms.sendStatus();
    r.sleep();
  }

  return 0;
}

