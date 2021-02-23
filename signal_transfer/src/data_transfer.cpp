
#include "data_transfer.h"

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "data_transfer");

    string topic_sub = "/darknet_ros/bounding_boxes";
    string topic_pub = "/perception/Detection/TrafficSign";
    
    //data trans(topic_sub);
    data trans(topic_sub, topic_pub);

    ros::spin();

};
