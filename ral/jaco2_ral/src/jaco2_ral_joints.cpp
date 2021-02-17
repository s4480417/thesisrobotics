#include <jaco2_ral/jaco2_ral_joints.hpp>
#include <jaco2_driver/jaco2_driver.hpp>

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <util/util.hpp>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>

double xl_track = 0.0;
double yl_track = 0.0;
double zl_track = 0.0;
double xr_track = 0.0;
double yr_track = 0.0;
double zr_track = 0.0;

bool flagl = false;
bool flagr = false;

void tracklCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    if (!flagl) {
        flagl = true;
    }
    xl_track = msg->x;
    yl_track = msg->y;
    zl_track = msg->z;
}
void trackrCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    if (!flagr) {
        flagr = true;
    }
    xr_track = msg->x;
    yr_track = msg->y;
    zr_track = msg->z;
}


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
    jaco2->home_arm();
    int i = 0;
    ros::NodeHandle nh_trackl;
    ros::Subscriber sub_trackl = nh_trackl.subscribe("usr/trackl", 100, tracklCallback);
    ros::NodeHandle nh_trackr;
    ros::Subscriber sub_trackr = nh_trackr.subscribe("usr/trackr", 100, trackrCallback);
    
    while (ros::ok()) {

        if (strcmp(argv[1], "left") == 0) {
            double position[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 3.141592 / 2.0};
            double current_position[6];
            if (flagl) {
                tf::Vector3 v0(xl_track, yl_track, zl_track);
                tf::Vector3 vL;
                origin_to_left(v0, &vL);
                position[0] = vL.x();
                position[1] = vL.y();
                position[2] = vL.z();
                if (v0.x() >= 0.75) {
                    jaco2->set_cartesian(position);
                }
                jaco2->clear_trajectories();
                jaco2->get_cartesian(current_position);
                double dist = sqrt(pow(position[0] - current_position[0], 2) + pow(position[1] - current_position[1], 2) + pow(position[2] - current_position[2], 2));
                double current_tau[6];
                jaco2->get_torques(current_tau);
                for (int i = 0; i < 6; i++) {
                    if (abs(current_tau[i]) > 20.0) {
                        std::cout << " WARNING! Joint " << i << " at limit." << std::endl;
                        jaco2->home_arm();
                    }
                }
            }
        }
        if (strcmp(argv[1], "right") == 0) {
            double position[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 3.141592 / 2.0};
            double current_position[6];
            if (flagr) {
                tf::Vector3 v0(xr_track, yr_track, zr_track);
                tf::Vector3 vR;
                origin_to_right(v0, &vR);
                position[0] = vR.x();
                position[1] = vR.y();
                position[2] = vR.z();
                if (v0.x() >= 0.75) {
                    jaco2->set_cartesian(position);
                }
                jaco2->clear_trajectories();
                jaco2->get_cartesian(current_position);
                double dist = sqrt(pow(position[0] - current_position[0], 2) + pow(position[1] - current_position[1], 2) + pow(position[2] - current_position[2], 2));
                double current_tau[6];
                jaco2->get_torques(current_tau);
                for (int i = 0; i < 6; i++) {
                    if (abs(current_tau[i]) > 20.0) {
                        jaco2->home_arm();
                    }
                }
            }
        }
        ros::Rate(5).sleep();
        ros::spinOnce();
        init = false;
    }
    delete jaco2;
    return 0;
}

