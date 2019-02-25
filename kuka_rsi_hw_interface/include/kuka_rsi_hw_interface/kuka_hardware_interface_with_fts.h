#ifndef KUKA_RSI_HARDWARE_INTERFACE_WITH_FTS
#define KUKA_RSI_HARDWARE_INTERFACE_WITH_FTS

#include <force_controllers/force_controller.h>
#include <force_torque_sensor/force_torque_sensor_handle.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

namespace kuka_rsi_hw_interface
{

class KukaHardwareInterfaceWithFTS : public KukaHardwareInterface
{
    force_torque_control::ForceTorqueControllerInterface ftc_interface_;
//     ForceTorqueSensorHandle* ftsh_;
    boost::shared_ptr<hardware_interface::ForceTorqueSensorHandle> ftsh_;
    ros::NodeHandle nh_;

    std::string fts_name;
    std::string fts_sensor_frame;
    std::string fts_transform_frame;

public:
    KukaHardwareInterfaceWithFTS(ros::NodeHandle nh);
//     ~KukaHardwareInterfaceWithFTS();
    
    void handleInit();
};
} // namespace kuka_rsi_hw_interface

#endif
