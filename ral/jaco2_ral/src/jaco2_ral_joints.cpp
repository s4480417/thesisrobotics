#include <jaco2_ral/jaco2_ral_joints.hpp>
#include <jaco2_driver/jaco2_driver.hpp>

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <util/util.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "jaco_ral_joints");
    boost::recursive_mutex api_mutex;
    bool init = true;
    Jaco2* jaco2;
    if (strcmp(argv[1], "left") == 0) {
        jaco2 = new Jaco2("left", "PJ00000001030703130", init);
    }
    if (strcmp(argv[1], "right") == 0) {
        jaco2 = new Jaco2("right", "PJ00000001030703133", init);
    }
    while (ros::ok()) {

        //double theta_left[6] = {90, 180, 270, 180, 270, 0};
        //double theta_right[6] = {270, 180, 90, 180, 90, 0};

        //double current_theta_left[6];
        //double current_theta_right[6];

        if (strcmp(argv[1], "left") == 0) {
            //jaco2->set_joints(theta_left);
            //jaco2->get_joints(current_theta_left);
            jaco2->set_cartesian()
        }
        if (strcmp(argv[1], "right") == 0) {
            //jaco2->set_joints(theta_right);
            //jaco2->get_joints(current_theta_right);
        }

        ros::Rate(2).sleep();
        ros::spinOnce();
        init = false;
    }
    delete jaco2;
    return 0;
}

