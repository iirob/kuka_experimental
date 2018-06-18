

#include <kuka_rsi_hw_interface/kuka_hardware_interface_with_fts.h>

namespace kuka_rsi_hw_interface
{
KukaHardwareInterfaceWithFTS::KukaHardwareInterfaceWithFTS(ros::NodeHandle nh) : KukaHardwareInterface(nh)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
            ros::console::notifyLoggerLevelsChanged();
    nh_ = nh;
    nh_.param<std::string>("/fts/FTS/name", fts_name, "fts_sensor_name");
    nh_.param<std::string>("/fts/Node/transform_frame", fts_transform_frame, "fts_base_link");
    ROS_DEBUG_STREAM("try to register the force torque controller interface to simulation env.");
    registerInterface(&ftc_interface_);
    ROS_DEBUG_STREAM("successfully registered force torque controller interface");
    
    // create a robot handle for kr5, and attach the fake sensor handle
    ROS_DEBUG_STREAM("creating ForceTorqueControllerHandle");
    force_torque_control::ForceTorqueControllerHandle handle("kr5_controller_handle");
    
    ROS_DEBUG_STREAM("creating ForceTorqueSensorHandle");
    ftsh_.reset(new force_torque_sensor::ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame));
    ROS_INFO_STREAM("ForceTorqueSensorHandle created!");
    
    handle.addHandle(*ftsh_);
    ROS_INFO("force torque sensor handle added");

    std::vector<std::string> pos_names = position_joint_interface_.getNames();
    for(size_t i = 0; i < pos_names.size(); i ++)
        handle.addPosHandle(position_joint_interface_.getHandle(pos_names[i]));
    ROS_DEBUG_STREAM("add position handle");
//     TODO add velocity interface to kuka_hardware_interface.h
    std::vector<std::string> vel_names = velocity_joint_interface_.getNames();
    for(size_t i = 0; i < vel_names.size(); i ++)
        handle.addVelHandle(velocity_joint_interface_.getHandle(vel_names[i]));
    ROS_DEBUG_STREAM("add velocity handle");
//     // when effort interface is supported: uncomment
//     std::vector<std::string> eff_names = eff_interface_.getNames();
//     for(size_t i = 0; i < eff_names.size(); i ++)
//         handle.addPosHandle(eff_interface_.getHandle(eff_names[i]));

    ftc_interface_.registerHandle(handle);
    ROS_INFO("register handle to ftc interface");    
}

} // namespace kuka_rsi_hw_interface
