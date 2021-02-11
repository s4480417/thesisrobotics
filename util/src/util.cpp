#include <ros/ros.h>

#include <iostream>

#include <tf/tf.h>
#include <util/util.hpp>


const float delta_Jx = 0.15675;
const float delta_Jy = 0.13335;
const float delta_Kx = 0.125;
const float delta_Kz = 0.3;

tf::Matrix3x3 R_0K(
    0, 0, 1,
    1, 0, 0,
    0, -1, 0
);
tf::Vector3 d_0K(delta_Kx, 0, delta_Kz);
tf::Transform T_0K = tf::Transform(R_0K, d_0K);

tf::Matrix3x3 R_0L(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0
);
tf::Vector3 d_0L(-delta_Jx, delta_Jy, 0);

tf::Transform T_0L = tf::Transform(R_0L, d_0L);

tf::Matrix3x3 R_0R(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0
);
tf::Vector3 d_0R(-delta_Jx, -delta_Jy, 0);

tf::Transform T_0R = tf::Transform(R_0R, d_0R);

void origin_to_kinect(tf::Vector3 v0, tf::Vector3* vK) {
    *vK = T_0K.invXform(v0);
}

void origin_to_left(tf::Vector3 v0, tf::Vector3* vL) {
    *vL = T_0L.invXform(v0);
}

void origin_to_right(tf::Vector3 v0, tf::Vector3* vR) {
    *vR = T_0R.invXform(v0);
}

void kinect_to_origin(tf::Vector3 vK, tf::Vector3* v0) {
    *v0 = T_0K(vK);
}

void left_to_origin(tf::Vector3 vL, tf::Vector3* v0) {
    *v0 = T_0L(vL);
}

void right_to_origin(tf::Vector3 vR, tf::Vector3* v0) {
    *v0 = T_0R(vR);
}