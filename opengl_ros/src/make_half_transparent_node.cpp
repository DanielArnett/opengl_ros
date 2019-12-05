#include "make_half_transparent_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_half_transparent");

    SimpleRendererNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
