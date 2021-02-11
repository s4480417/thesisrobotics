#ifndef UTIL_HPP
#define UTIL_HPP

void origin_to_kinect(tf::Vector3 v0, tf::Vector3* vK);

void origin_to_left(tf::Vector3 v0, tf::Vector3* vL);

void origin_to_right(tf::Vector3 v0, tf::Vector3* vR);

void kinect_to_origin(tf::Vector3 vK, tf::Vector3* v0);

void left_to_origin(tf::Vector3 vL, tf::Vector3* v0);

void right_to_origin(tf::Vector3 vR, tf::Vector3* v0);

#endif