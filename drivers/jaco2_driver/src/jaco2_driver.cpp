#include <jaco2_driver/jaco2_driver.hpp>
#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_arm.h>
#include <kinova_driver/kinova_ros_types.h>


#include <string>

Jaco2::Jaco2(std::string _arm, std::string _serial_number, bool init) {
    std::string arm = _arm;
    std::string serial_number = _serial_number;
    std::string prefix = "j2n6s300";
    nh = new ros::NodeHandle(arm + "_driver");
    nh->setParam(nh->getNamespace() + "/serial_number", serial_number.c_str());
    nh->setParam(nh->getNamespace() + "/robot_name", arm.c_str());
    nh->setParam(nh->getNamespace() + "/robot_type", "j2n6s300");
    nh->setParam(nh->getNamespace() + "/use_jaco_v1_fingers", "false");
    nh->setParam(nh->getNamespace() + "/status_interval_seconds", "0.1");

    comm = new kinova::KinovaComm(*nh, api_mutex, init, prefix);
    kinova_arm = new kinova::KinovaArm(*comm, *nh, prefix, arm);
}


Jaco2::~Jaco2() {
    delete nh;
    delete comm;
    delete kinova_arm;
}

void Jaco2::set_joints(double theta[6]) {
    kinova::KinovaAngles angles;
    angles.Actuator1 = theta[0];
    angles.Actuator2 = theta[1];
    angles.Actuator3 = theta[2];
    angles.Actuator4 = theta[3];
    angles.Actuator5 = theta[4];
    angles.Actuator6 = theta[5];
    angles.Actuator7 = 0.0;
    comm->setJointAngles(angles, 30.0, 30.0, 0.0, true);
}

void Jaco2::get_joints(double current_theta[6]) {
    kinova::KinovaAngles current_angles;
    comm->getJointAngles(current_angles);
    current_theta[0] = current_angles.Actuator1;
    current_theta[1] = current_angles.Actuator2;
    current_theta[2] = current_angles.Actuator3;
    current_theta[3] = current_angles.Actuator4;
    current_theta[4] = current_angles.Actuator5;
    current_theta[5] = current_angles.Actuator6;
}

void Jaco2::get_torques(double current_tau[6]) {
    kinova::KinovaAngles current_torques;
    comm->getJointTorques(current_torques);
    current_tau[0] = current_torques.Actuator1;
    current_tau[1] = current_torques.Actuator2;
    current_tau[2] = current_torques.Actuator3;
    current_tau[3] = current_torques.Actuator4;
    current_tau[4] = current_torques.Actuator5;
    current_tau[5] = current_torques.Actuator6;
}

void Jaco2::set_cartesian(double position[6]) {
    kinova::KinovaPose pose;
    pose.X = position[0];
    pose.Y = position[1];
    pose.Z = position[2];
    pose.ThetaX = position[3];
    pose.ThetaY = position[4];
    pose.ThetaZ = position[5];
    try {
        comm->setCartesianPosition(pose, false);
    } catch (kinova::KinovaCommException e) {
        std::cerr << "Failed." << std::endl;
        clear_trajectories();
    }
}

void Jaco2::get_cartesian(double current_position[6]) {
    kinova::KinovaPose current_pose;
    comm->getCartesianPosition(current_pose);
    current_position[0] = current_pose.X;
    current_position[1] = current_pose.Y;
    current_position[2] = current_pose.Z;
    current_position[3] = current_pose.ThetaX;
    current_position[4] = current_pose.ThetaY;
    current_position[5] = current_pose.ThetaZ;
}

bool Jaco2::is_stopped() {
    return comm->isStopped();
}

void Jaco2::home_arm() {
    comm->homeArm();
}

void Jaco2::clear_trajectories() {
    comm->eraseAllTrajectories();
}

void Jaco2::set_fingers(double value) {
    kinova::FingerAngles fingers;
    fingers.Finger1 = value * 6800;
    fingers.Finger2 = value * 6800;
    fingers.Finger3 = value * 6800;
    comm->setFingerPositions(fingers, 100, true);
}