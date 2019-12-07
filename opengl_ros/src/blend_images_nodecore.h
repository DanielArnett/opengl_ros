#ifndef SIMPLE_RENDERER_NODECORE_H
#define SIMPLE_RENDERER_NODECORE_H

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "simple_renderer.h"
#include <cv_bridge/cv_bridge.h>

namespace opengl_ros {

class SimpleRendererNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::ImageTransport second_it_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;
    image_transport::Subscriber imageSubscriber_;
    image_transport::Subscriber imageSecondSubscriber_;
    
    //Other members
    std::unique_ptr<cgs::SimpleRenderer> renderer_;
    cv::Mat output_;
    cv_bridge::CvImagePtr first_cv_ptr;
    cv_bridge::CvImagePtr second_cv_ptr;
    bool secondImageInit_ = false;


    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageSecondCallback(const sensor_msgs::Image::ConstPtr& msg);

public:
    SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif
