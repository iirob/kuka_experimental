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

#include <hardware_interface/robot_hw.h>
#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>
#include <kuka_rsi_hw_interface/kuka_hardware_interface_with_fts.h>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/shared_ptr.hpp>

boost::shared_ptr<kuka_rsi_hw_interface::KukaHardwareInterface> kuka_rsi_hwi_;
boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

ros::Time timestamp_;
ros::Duration period_;
auto stopwatch_last_ = std::chrono::steady_clock::now();
auto stopwatch_now_ = std::chrono::steady_clock::now();

void updateRSIData(const ros::TimerEvent& event)
{

    // Receive current state from robot
    if (!kuka_rsi_hwi_->read(timestamp_, period_))
    {
        ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down!");
        ros::shutdown();
    }
    
    // Get current time and elapsed time since last read
    timestamp_ = ros::Time::now();
    stopwatch_now_ = std::chrono::steady_clock::now();
    period_.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now_ - stopwatch_last_).count());
    stopwatch_last_ = stopwatch_now_;

    // Update the controllers
    controller_manager_->update(timestamp_, period_);

    // Send new setpoint to robot
    kuka_rsi_hwi_->write(timestamp_, period_);
}


int main(int argc, char** argv)
{
    ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");

    ros::init(argc, argv, "kuka_rsi_hardware_interface");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;
        
    bool kuka_fts;
 
    if (nh.getParam("kuka_fts", kuka_fts) and kuka_fts) 
    {
        ROS_INFO_STREAM_NAMED("hardware_interface", "This is KUKA hardware interface with FTS!");
        kuka_rsi_hwi_.reset(new kuka_rsi_hw_interface::KukaHardwareInterfaceWithFTS(nh));
    }
    else 
    {
        ROS_INFO_STREAM_NAMED("hardware_interface", "This is KUKA hardware interface without FTS!");
        kuka_rsi_hwi_.reset(new kuka_rsi_hw_interface::KukaHardwareInterface(nh));
    }
    kuka_rsi_hwi_->configure();
    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    // TODO: controller manager takes only hardware_interface::RobotHW* as arguments
    controller_manager_.reset(new controller_manager::ControllerManager(kuka_rsi_hwi_.get(), nh));
    kuka_rsi_hwi_->start();
    ROS_WARN("kuka_rsi_hwi successfully started!");

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;
    // time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;
    
    // Get current time and elapsed time since last read
    timestamp_ = ros::Time::now();
    stopwatch_now_ = std::chrono::steady_clock::now();
    period_.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now_ - stopwatch_last_).count());
    stopwatch_last_ = stopwatch_now_;
    
    ros::Timer timer = nh.createTimer(ros::Duration(ros::Rate(1000)), updateRSIData);
    
    

//     // Run as fast as possible
//     while (ros::ok())
//     //while (!g_quit)
//     {
//         // Receive current state from robot
//         if (!kuka_rsi_hwi_->read(timestamp, period))
//         {
//             ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down!");
//             ros::shutdown();
//         }
//         
//         // Get current time and elapsed time since last read
//         timestamp = ros::Time::now();
//         stopwatch_now = std::chrono::steady_clock::now();
//         period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
//         stopwatch_last = stopwatch_now;
// 
//         // Update the controllers
//         controller_manager.update(timestamp, period);
// 
//         // Send new setpoint to robot
//         kuka_rsi_hwi_->write(timestamp, period);
//     }

//     spinner.stop();
    ros::waitForShutdown();
    ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

    return 0;

}
