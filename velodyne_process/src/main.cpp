#include<perception.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_process");
    ros::NodeHandle node;
    Perception PCT(node);

    ros::spin();

    return 0;
}
