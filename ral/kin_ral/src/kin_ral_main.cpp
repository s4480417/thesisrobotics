#include <ros/ros.h>
#include <kin_driver/kin_driver.hpp>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include <std_msgs/Float64.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <util/util.hpp>

#define NUM_KEYPOINTS 19

#define COLOR_WIDTH     1920
#define COLOR_HEIGHT    1080
#define IR_WIDTH        512
#define IR_HEIGHT       424
#define DEPTH_WIDTH     512
#define DEPTH_HEIGHT    424

enum BodyPart {
    NOSE        = 0,
    NECK        = 1,
    R_SHOULDER  = 2,
    R_ELBOW     = 3,
    R_WRIST     = 4,
    L_SHOULDER  = 5,
    L_ELBOW     = 6,
    L_WRIST     = 7,
    R_HIP       = 8,
    R_KNEE      = 9,
    R_ANKLE     = 10,
    L_HIP       = 11,
    L_KNEE      = 12,
    L_ANKLE     = 13,
    R_EYE       = 14,
    L_EYE       = 15,
    R_EAR       = 16,
    L_EAR       = 17,
    BKG         = 18
};

class UsrBody {
private:
    tf::Vector3 positions[NUM_KEYPOINTS];
    bool set_flags[NUM_KEYPOINTS];
public:
    void setPosition(BodyPart bp, tf::Vector3 position, double weight);
    tf::Vector3 getPosition(BodyPart bp);
};

UsrBody body;
libfreenect2::Frame undistorted(DEPTH_WIDTH, DEPTH_HEIGHT, 4);
libfreenect2::Frame registered(DEPTH_WIDTH, DEPTH_HEIGHT, 4);
libfreenect2::Registration* registration;

tf::Vector3 UsrBody::getPosition(BodyPart bp) {
    return positions[bp];
}

void UsrBody::setPosition(BodyPart bp, tf::Vector3 position, double weight) {
    
    if (!set_flags[bp]) {
        set_flags[bp] = true;
        positions[bp] = position;
    } else if (positions[bp].distance(position) < 0.5) {
        positions[bp] = position * weight + positions[bp] * (1.0 - weight);
        return;
    } else {
        set_flags[bp] = false;
    }
}

std::string to_string(BodyPart bp) {
    switch (bp) {
        case NOSE:
            return std::string("Nose");
        case NECK:
            return std::string("Neck");
        case R_SHOULDER:
            return std::string("Right Shoulder");
        case R_ELBOW:
            return std::string("Right Elbow");
        case R_WRIST:
            return std::string("Right Wrist");
        case L_SHOULDER:
            return std::string("Left Shoulder");
        case L_ELBOW:
            return std::string("Left Elbow");
        case L_WRIST:
            return std::string("Left Wrist");
        case R_HIP:
            return std::string("Right Hip");
        case R_KNEE:
            return std::string("Right Knee");
        case R_ANKLE:
            return std::string("Right Ankle");
        case L_HIP:
            return std::string("Left Hip");
        case L_KNEE:
            return std::string("Left Knee");
        case L_ANKLE:
            return std::string("Left Ankle");
        case R_EYE:
            return std::string("Right Eye");
        case L_EYE:
            return std::string("Left Eye");
        case R_EAR:
            return std::string("Right Ear");
        case L_EAR:
            return std::string("Left Ear");
        case BKG:
            return std::string("Background");
        default:
            return std::string("");
    }
}

ros::Publisher pub_pose_viz;

void messageCallback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg) {
    if (msg->num_humans > 0) {
        for (int i = 0; i < msg->num_humans; i++) {
            for (int j = 0; j < NUM_KEYPOINTS; j++) {
                openpose_ros_msgs::PointWithProb point = msg->human_list[i].body_key_points_with_prob[j];
                tf::Vector3 position(point.x, point.y, 0.0);
                BodyPart bp = static_cast<BodyPart>(j);
                body.setPosition(bp, position, 0.5);
            }
        }
        pcl::PointCloud<pcl::PointXYZ> pose_viz;
        for (int i = 2; i < 14; i++) {
            BodyPart bp = static_cast<BodyPart>(i);
            tf::Vector3 position = body.getPosition(bp);
            int c = (int)position.x();
            int r = (int)position.y();
            pcl::PointXYZ point;
            point.x = 0.0;
            point.y = 0.0;
            point.z = 0.0;
            int count = 0;
            for (int c_off = -2; c_off <= 2; c_off++) {
                for (int r_off = -2; r_off <= 2; r_off++) {
                    float x_raw = 0;
                    float y_raw = 0;
                    float z_raw = 0;
                    registration->getPointXYZ(&undistorted, r + r_off, c + r_off, y_raw, z_raw, x_raw);
                    if (!isnan(x_raw) && !isnan(y_raw) && !isnan(z_raw)) {
                        z_raw *= -1;
                        point.x += x_raw;
                        point.y += y_raw;
                        point.z += z_raw;
                        count++;
                    }
                }
            }
            if (count != 0) {
                point.x /= (float)count;
                point.y /= (float)count;
                point.z /= (float)count;
            }
            pose_viz.points.push_back(point);
        }
        sensor_msgs::PointCloud2 pose_viz_msg;
        pcl::toROSMsg(pose_viz, pose_viz_msg);
        pose_viz_msg.header.frame_id = "kinect";
        pose_viz_msg.header.stamp = ros::Time::now();
        pub_pose_viz.publish(pose_viz_msg);
    }
}

libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

int main(int argc, char** argv) {

    ros::init(argc, argv, "kinect_main");
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    pipeline = new libfreenect2::OpenGLPacketPipeline();
    if (freenect2.enumerateDevices() == 0) {
        std::cout << "No device connected!" << std::endl;
        return -1;
    }
    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    if (pipeline) {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }
    if (dev == 0) {
        std::cout << "Failure opening device" << std::endl;
        return -1;
    }
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    libfreenect2::Freenect2Device::Config config;
    config.MinDepth = 0.0;
    config.MaxDepth = 8.0;
    dev->setConfiguration(config);
    if (!dev->start()) {
        return -1;
    }
    
    std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();
    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();

    dev->setColorCameraParams(color_params);
    dev->setIrCameraParams(ir_params);

    registration = new libfreenect2::Registration(ir_params, color_params);

    
    ros::NodeHandle nh_registered;
    ros::NodeHandle nh_undistorted;
    ros::NodeHandle nh_registered_raw;
    ros::NodeHandle nh_undistorted_raw;

    ros::NodeHandle nh_pose;
    ros::NodeHandle nh_cloud;
    ros::NodeHandle nh_pose_viz;

    image_transport::ImageTransport it_registered(nh_registered);
    image_transport::ImageTransport it_undistorted(nh_undistorted);
    image_transport::ImageTransport it_registered_raw(nh_registered_raw);
    image_transport::ImageTransport it_undistorted_raw(nh_undistorted_raw);


    image_transport::Publisher pub_registered = it_registered.advertise("kin/img_registered", 1);
    image_transport::Publisher pub_undistorted = it_undistorted.advertise("kin/img_undistorted", 1);
    image_transport::Publisher pub_registered_raw = it_registered_raw.advertise("kin/img_registered_raw", 1);
    image_transport::Publisher pub_undistorted_raw = it_undistorted_raw.advertise("kin/img_undistorted_raw", 1);

    ros::Subscriber sub_pose = nh_pose.subscribe("usr/keypoints", 100, messageCallback);
    ros::Publisher pub_cloud = nh_cloud.advertise<sensor_msgs::PointCloud2>("viz/points", 1);
    pub_pose_viz = nh_pose_viz.advertise<sensor_msgs::PointCloud2>("viz/pose", 1);

    while (ros::ok()) {
        if (!listener.waitForNewFrame(frames, 10*10000)) {
            std::cout << "Timeout!" << std::endl;
            return -1;
        }

        libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        
        registration->apply(color, depth, &undistorted, &registered);
        registration->undistortDepth(depth, &undistorted);

        Mat img_color = Mat(Size(color->width, color->height), CV_8UC4);
        Mat img_ir = Mat(Size(ir->width, ir->height), CV_8UC1);
        Mat img_depth = Mat(Size(depth->width, depth->height), CV_32FC1);
        
        memcpy(img_color.data, color->data, color->width * color->height * sizeof(unsigned char) * 4);
        memcpy(img_ir.data, ir->data, ir->width * ir->height * sizeof(unsigned char));
        memcpy(img_depth.data, depth->data, depth->width * depth->height * sizeof(float));
        
        Mat img_registered = Mat(Size(registered.width, registered.height), CV_8UC4);
        Mat img_undistorted = Mat(Size(undistorted.width, undistorted.height), CV_32FC1);

        
        memcpy(img_registered.data, registered.data, registered.width * registered.height * sizeof(unsigned char) * 4);
        memcpy(img_undistorted.data, undistorted.data, undistorted.width * undistorted.height * sizeof(float));

        cvtColor(img_registered, img_registered, CV_RGBA2RGB);
        
        Mat img_mask;
        
        compare(Scalar(0, 0, 0, 0), img_registered, img_mask, CMP_EQ);
        cvtColor(img_mask, img_mask, CV_RGBA2GRAY);

        std::for_each(img_undistorted.begin<float>(), img_undistorted.end<float>(), func);
        img_undistorted.convertTo(img_undistorted, CV_8UC1);
        
        Mat img_registered_raw;
        Mat img_undistorted_raw;
        img_registered.copyTo(img_registered_raw);
        img_undistorted.copyTo(img_undistorted_raw);

        medianBlur(img_undistorted, img_undistorted, 5);
        Mat mask;
        threshold(img_undistorted, mask, 0, 255, THRESH_OTSU | THRESH_BINARY);
        Mat foreground_color;
        Mat foreground_depth;
        img_registered.copyTo(foreground_color, mask);
        img_undistorted.copyTo(foreground_depth, mask);

        sensor_msgs::ImagePtr msg_registered = cv_bridge::CvImage(std_msgs::Header(), "bgr8", foreground_color).toImageMsg();
        sensor_msgs::ImagePtr msg_undistorted = cv_bridge::CvImage(std_msgs::Header(), "mono8", foreground_depth).toImageMsg();
        sensor_msgs::ImagePtr msg_registered_raw = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_registered_raw).toImageMsg();
        sensor_msgs::ImagePtr msg_undistorted_raw = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_undistorted_raw).toImageMsg();

        pub_registered.publish(msg_registered);
        pub_undistorted.publish(msg_undistorted);
        pub_registered_raw.publish(msg_registered_raw);
        pub_undistorted_raw.publish(msg_undistorted_raw);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (int i = 0; i < depth->height; i++) {
            for (int j = 0; j < depth->width; j++ ) {
                float d = ((float*)(undistorted.data))[i * depth->width + j];

                pcl::PointXYZRGB point;
                float rgb;
                
                float x_raw = 0;
                float y_raw = 0;
                float z_raw = 0;
                registration->getPointXYZRGB(&undistorted, &registered, i, j, x_raw, y_raw, z_raw, rgb);
                
                tf::Vector3 v_raw(x_raw, y_raw, z_raw);
                tf::Vector3 v_new;
                kinect_to_origin(v_raw, &v_new);

                point.x = v_new.x();
                point.y = v_new.y();
                point.z = v_new.z();

                const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
                point.b = p[0];
                point.g = p[1];
                point.r = p[2];
                if (mask.at<uchar>(i, j) != 0) {
                    cloud.points.push_back(point);
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "root";
        cloud_msg.header.stamp = ros::Time::now();
        pub_cloud.publish(cloud_msg);
        ros::spinOnce();
        
        listener.release(frames);
        waitKey(1);
    }
    dev->stop();
    dev->close();
    
    return 0;
}
