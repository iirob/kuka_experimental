#ifndef KUKA_RSI_HARDWARE_INTERFACE_WITH_FTS
#define KUKA_RSI_HARDWARE_INTERFACE_WITH_FTS

#include <boost/shared_ptr.hpp>

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

#include <force_torque_sensor/force_torque_sensor_handle.h>
#include <hardware_interface/force_torque_sensor_interface.h>

namespace kuka_rsi_hw_interface {

    class KukaHardwareInterfaceWithFTS : public KukaHardwareInterface {

        hardware_interface::ForceTorqueSensorInterface fts_interface_;
        boost::shared_ptr<hardware_interface::ForceTorqueSensorHandle> ftsh_;

    public:

        explicit KukaHardwareInterfaceWithFTS(ros::NodeHandle nh);

    };
} // namespace kuka_rsi_hw_interface

#endif
