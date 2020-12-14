/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>


namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface() :
    joint_position_(6, 0.0), joint_position_last_(6, 0.0), joint_velocity_(6, 0.0), joint_effort_(6, 0.0),
    joint_position_command_(6, 0.0), joint_velocity_command_(6, 0.0), joint_effort_command_(6, 0.0), joint_names_(6),
    rsi_initial_joint_positions_(6, 0.0), rsi_joint_position_corrections_(6, 0.0), ipoc_(0), n_dof_(6)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }

  //Create ros_control interfaces
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");
}

KukaHardwareInterface::~KukaHardwareInterface()
{

}

void KukaHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return;
  }

  if (rt_rsi_pub_->trylock()){
    rt_rsi_pub_->msg_.data = in_buffer_;
    rt_rsi_pub_->unlockAndPublish();
  }




  rsi_state_ = RSIState(in_buffer_);
  if(rsi_state_.ipoc > ipoc_)
  {
    control_period_.fromSec(krc_multiplier_ * (rsi_state_.ipoc - ipoc_) / 1000.0);
  }

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    if(!period.isZero())
    {
      joint_velocity_[i] = (joint_position_[i] - joint_position_last_[i]) / period.toSec();
    }
  }
  ipoc_ = rsi_state_.ipoc;
  joint_position_last_ = joint_position_;

  if (rt_ain_pub_->trylock()){
    rt_ain_pub_->msg_.data = rsi_state_.analog_in;
    rt_ain_pub_->unlockAndPublish();
  }

  if (rt_din_pub_->trylock()){
    rt_din_pub_->msg_.data = rsi_state_.digital_in;
    rt_din_pub_->unlockAndPublish();
  }

  if (rt_dout_pub_->trylock()){
    rt_dout_pub_->msg_.data = rsi_state_.digital_out;
    rt_dout_pub_->unlockAndPublish();
  }

  if (rt_fts_pub_->trylock()){
    std::vector<double> fts(6, 0.0);
    //<Axis Name="Fx" values=" -0.14551  -0.06633  -3.05426 -39.06492   0.89394  33.87097 " max="1000" scale="0.390958596724837"/>
		//<Axis Name="Fy" values="  1.43959  40.38336  -1.65248 -22.57362   0.39793 -19.58281 " max="1000" scale="0.390958596724837"/>
		//<Axis Name="Fz" values=" 17.78615  -0.51980  18.90878  -0.54382  18.46395  -0.81585 " max="2500" scale="0.143188586050472"/>
		//<Axis Name="Tx" values="  0.00670   0.15123 -33.14474   1.09359  32.86976  -1.65849 " max="120" scale="3.98269830325007"/>
		//<Axis Name="Ty" values=" 36.20237  -1.27033 -18.89249   0.94801 -19.28218   0.73598 " max="120" scale="3.98269830325007"/>
		//<Axis Name="Tz" values=" -1.34822 -19.22028  -1.17290 -21.48861   0.04581 -18.75570 " max="120" scale="3.6363767116631"/>

    double a1 = rsi_state_.analog_in[0];
    double a2 = rsi_state_.analog_in[1];
    double a3 = rsi_state_.analog_in[2];
    double a4 = rsi_state_.analog_in[3];
    double a5 = rsi_state_.analog_in[4];
    double a6 = rsi_state_.analog_in[5];

    fts[0] = -0.14551 * a1 + -0.06633 * a2 + -3.05426 * a3 + -39.06492 * a4 + 0.89394 * a5 + 33.87097 * a6;
    fts[1] = 1.43959 * a1 + 40.38336 * a2 + -1.65248 * a3 + -22.57362 * a4 + 0.39793 * a5 + -19.58281 * a6;
    fts[2] = 17.78615 * a1 + -0.51980 * a2 + 18.90878 * a3 + -0.54382 * a4 + 18.46395 * a5 + -0.81585 * a6;
    fts[3] = 0.00670 * a1 + 0.15123 * a2 + -33.14474  * a3 + 1.09359  * a4 + 32.86976  * a5 + -1.65849 * a6;
    fts[4] = 36.20237 * a1 + -1.27033 * a2 + -18.89249 * a3 + 0.94801 * a4 + -19.28218 * a5 + 0.73598 * a6;
    fts[5] = -1.34822 * a1 + -19.22028 * a2 + -1.17290 * a3 + -21.48861 * a4 + 0.04581 * a5 + -18.75570 * a6;

    rt_fts_pub_->msg_.data = fts;
    rt_fts_pub_->unlockAndPublish();
  }
  

}

void KukaHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);
}

void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
    krc_multiplier_ = 12.0;
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");

}

void KukaHardwareInterface::configure()
{
  const std::string param_addr = "rsi/listen_address";
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
    " parameter server (looking for '" + param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
  rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
  rt_ain_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh_, "analog_in", 3));
  rt_din_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32MultiArray>(nh_, "digital_in", 3));
  rt_dout_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32MultiArray>(nh_, "digital_out", 3));
  rt_fts_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh_, "FTS", 3));

  
}

bool KukaHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  configure();
  start();
  return true;
}

} // namespace kuka_rsi_hardware_interface

PLUGINLIB_EXPORT_CLASS(kuka_rsi_hw_interface::KukaHardwareInterface, hardware_interface::RobotHW)
