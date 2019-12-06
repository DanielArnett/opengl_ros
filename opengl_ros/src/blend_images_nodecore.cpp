#include "blend_images_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh), second_it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);
    imageSubscriber_ = it_.subscribe("image_in" , 1, &SimpleRendererNode::imageCallback, this);
    imageSecondSubscriber_ = second_it_.subscribe("second_image_in" , 1, &SimpleRendererNode::imageSecondCallback, this);
    
    int width, height;
    nh_.param<int>("width" , width , 640);
    nh_.param<int>("height", height, 480);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");

    renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width, height, 
        vertexShader, fragmentShader
    );

    output_.create(height, width, CV_8UC4);
    secondImage_.create(height, width, CV_8UC4);
}

void SimpleRendererNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    auto start = std::chrono::system_clock::now();
//     TODO add a second texture
//    renderer_->AddTexture("secondImage", secondImage_, 1);
    const auto& image = cv_ptr->image;
    renderer_->render(output_, image, secondImage_);

    ROS_DEBUG_STREAM(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() << "ns";
    );
    cv::imwrite("/home/NEA.com/daniel.arnett/Pictures/image_1.png", image);
    //Publish
    cv_bridge::CvImage outImage;
    outImage.header = cv_ptr->header;
    outImage.encoding = cv_ptr->encoding;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void SimpleRendererNode::imageSecondCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    secondImage_ = cv_ptr->image;
    cv::imwrite("/home/NEA.com/daniel.arnett/Pictures/image_2.png", secondImage_);
}
void SimpleRendererNode::run()
{
    ros::spin();
}
