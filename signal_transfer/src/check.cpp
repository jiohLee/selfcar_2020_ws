
#include "data_transfer.h"

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "check");
    string topic_sub = "/perception/Detection/TrafficSign";
    
    data trans(topic_sub, 1);
    ros::spin();

};
