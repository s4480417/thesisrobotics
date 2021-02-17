#ifndef JACO2_DRIVER_HPP
#define JACO2_DRIVER_HPP

#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_arm.h>
#include <string>

class Jaco2 {
    kinova::KinovaComm* comm;
    kinova::KinovaArm* kinova_arm;
    ros::NodeHandle* nh;
    boost::recursive_mutex api_mutex;
public:
    Jaco2(std::string _arm, std::string _serial_number, bool init);
    ~Jaco2();
    void set_joints(double theta[6]);
    void get_joints(double current_theta[6]);
    void set_cartesian(double position[6]);
};



#endif