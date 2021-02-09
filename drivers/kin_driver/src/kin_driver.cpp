#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#include <kin_driver/kin_driver.hpp>

using namespace cv;

void func(float& e) {
     e = (e - 500.0) / 1500.0;
     if (e <= 1e-3) {
         e = 1.0;
     }
     e = 255.0 * (1.0 - e);
}

libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

std::string Kinect::getSerialNumber() {
    return dev->getSerialNumber();
}

std::string Kinect::getFirmwareVersion() {
    return dev->getFirmwareVersion();
}

int Kinect::start() {
    if (!dev->start()) {
        return -1;
    }
    return 0;
}

int Kinect::stop() {
    dev->stop();
    dev->close();
    return 0;
}

void Kinect::updateStream() {
    libfreenect2::FrameMap frames;
    // Wait for the next frames without a timeout
    if (!listener.waitForNewFrame(frames, 10*10000)) {
        std::cout << "Timeout!" << std::endl;
        return;
    }

    // Frames for Color, Infrared and Depth 
    libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // Combine data from color and depth images
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    registration->apply(color, depth, &undistorted, &registered);

    // Create OpenCV Mat objects
    Mat img_raw_color = Mat(Size(color->width, color->height), CV_8UC4, color->data);
    Mat img_raw_ir = Mat(Size(ir->width, ir->height), CV_8UC1, ir->data);
    Mat img_raw_depth = Mat(Size(depth->width, depth->height), CV_32FC1, depth->data);
    Mat img_registered = Mat(Size(registered.width, registered.height), CV_8UC4, registered.data);
    Mat img_undistorted = Mat(Size(undistorted.width, undistorted.height), CV_32FC1, undistorted.data);

    // Convert RGBX (Kinect Format) to BGRA to BGR (OpenCV Format)
    cvtColor(img_registered, img_registered, CV_BGRA2BGR);
    cvtColor(img_registered, img_registered, CV_BGRA2BGR);

    // Apply Transformation for Visualization and convert to 8-bit grayscale
    std::for_each(img_undistorted.begin<float>(), img_undistorted.end<float>(), func);
    img_undistorted.convertTo(img_undistorted, CV_8UC1);

    /* Remove Borders */
    Rect roi;
    roi.x = 0;
    roi.y = 50;
    roi.width = img_raw_depth.size().width;
    roi.height = img_raw_depth.size().height - 100;
    img_registered = img_registered(roi);
    img_undistorted = img_undistorted(roi);
    

    /* Remove S&P Noise with Median Filter */
    medianBlur(img_undistorted, img_undistorted, 5);


    /* Use Otsu's Method to Remove Background */
    Mat mask;
    threshold(img_undistorted, mask, 0, 255, THRESH_OTSU | THRESH_BINARY);

    Mat img_foreground_color;
    Mat img_foreground_depth;
    img_registered.copyTo(img_foreground_color, mask);
    img_undistorted.copyTo(img_foreground_depth, mask);
        
    listener.release(frames);

    img_raw_depth.copyTo(raw_depth);
    img_raw_color.copyTo(raw_color);

    img_foreground_color.copyTo(foreground_color);
    img_foreground_depth.copyTo(foreground_depth);
    
}

Kinect::Kinect() {
    pipeline = new libfreenect2::OpenGLPacketPipeline();
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    if (freenect2.enumerateDevices() == 0) {
        std::cout << "No device connected!" << std::endl;
        return;
    }
    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    if (pipeline) {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }
    if (dev == 0) {
        std::cout << "Failure opening device" << std::endl;
        return;
    }
    
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
}

int foo(int x) {
    std::cout << "Hello world!" << std::endl;
    std::cout  << std::to_string(x) << std::endl;
    return 0;
}
