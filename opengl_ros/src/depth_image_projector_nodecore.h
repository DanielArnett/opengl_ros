#ifndef DEPTH_IMAGE_PROJECTOR_NODECORE_H
#define DEPTH_IMAGE_PROJECTOR_NODECORE_H

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "depth_image_projector.h"

namespace opengl_ros {

class DepthImageProjectorNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;

    image_transport::CameraSubscriber colorSubscriber_;
    image_transport::CameraSubscriber depthSubscriber_;
    
    //Other members
    std::unique_ptr<cgs::DepthImageProjector> projector_;
    cv::Mat output_;

    cv_bridge::CvImageConstPtr      latestColorImagePtr;
    sensor_msgs::CameraInfoConstPtr latestColorCameraInfoPtr;
    void colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);

public:
    DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif