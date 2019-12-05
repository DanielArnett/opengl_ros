#include "check_transparency_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_transparency");

    SimpleRendererNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
