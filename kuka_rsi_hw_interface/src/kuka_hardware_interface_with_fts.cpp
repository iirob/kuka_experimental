#include <kuka_rsi_hw_interface/kuka_hardware_interface_with_fts.h>

namespace kuka_rsi_hw_interface {

    KukaHardwareInterfaceWithFTS::KukaHardwareInterfaceWithFTS(ros::NodeHandle nh) : KukaHardwareInterface(nh) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
            ros::console::notifyLoggerLevelsChanged();

        std::string fts_name, fts_transform_frame;
        nh.param<std::string>("/fts/FTS/name", fts_name, "fts_sensor_name");
        nh.param<std::string>("/fts/Node/transform_frame", fts_transform_frame, "fts_base_link");

        ftsh_.reset(new force_torque_sensor::ForceTorqueSensorHandle(nh, fts_name, fts_transform_frame));

        fts_interface_.registerHandle(*ftsh_);
        registerInterface(&fts_interface_);
    }
}