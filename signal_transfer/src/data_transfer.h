
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

using namespace std;
using namespace ros;

class data
{
private:
    ros::NodeHandle nh;
    ros::Publisher ros_pub;
    ros::Subscriber ros_sub;
    ros::Subscriber ros_sub2;
    
    darknet_ros_msgs::BoundingBoxes box;
    string R; string G; string LG; string LR; 
    std_msgs::String trans;

public:
    data();
    data( string sub );
    data( string sub, string pub);
    data( string sub, int n );

    void imageCallback_bounding( const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void imageCallback_bounding_pub( const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void stringCallback( const std_msgs::String::ConstPtr& msg );
};

data::data(){}

data::data( string sub )
{
    ros_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>( sub, 100, &data::imageCallback_bounding, this );
    R = "RED"; G = "GREEN"; LG = "LEFT_GREEN"; LR = "LEFT_RED";
}

data::data( string sub, string pub )
{
    R = "RED"; G = "GREEN"; LG = "LEFT_GREEN"; LR = "LEFT_RED";
    ros_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>( sub, 100, &data::imageCallback_bounding_pub, this );
    ros_pub = nh.advertise<std_msgs::String>(pub, 100);
}

data::data( string sub, int n )
{
    ros_sub = nh.subscribe<std_msgs::String>( sub, 100, &data::stringCallback, this );
}

//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

void data::imageCallback_bounding( const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg )
{
    int num = msg->bounding_boxes.size();
    for( int i = 0; i < num; i++ )
    {
        if( msg->bounding_boxes[i].id == 0 )
            ROS_INFO("%s", G.c_str() );
        else if( msg->bounding_boxes[i].id == 1)
            ROS_INFO("%s", R.c_str() ); 
        else if( msg->bounding_boxes[i].id == 2)
            ROS_INFO("%s", LG.c_str() ); 
        else if( msg->bounding_boxes[i].id == 3)
            ROS_INFO("%s", LR.c_str() ); 
        else if( msg->bounding_boxes[i].id == 4)
            ROS_INFO("%s", R.c_str() ); 
    }

}

void data::imageCallback_bounding_pub( const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg )
{
    int num = msg->bounding_boxes.size();
    for( int i = 0; i < num; i++ )
    {
        if( msg->bounding_boxes[i].id == 0 )
        {
            ROS_INFO("%s", G.c_str() ); trans.data = G;
        }
        else if( msg->bounding_boxes[i].id == 1  )
        {
            ROS_INFO("%s", R.c_str() ); trans.data = R;
        }
        else if( msg->bounding_boxes[i].id == 2  )
        {
            ROS_INFO("%s", LG.c_str() ); trans.data = LG;
        }
        else if( msg->bounding_boxes[i].id == 3  )
        {
            ROS_INFO("%s", LR.c_str() ); trans.data = LR;
        }
        else if( msg->bounding_boxes[i].id == 4  )
        {
            ROS_INFO("%s", R.c_str() ); trans.data = R;
        }
        ros_pub.publish(trans);
    }
}

void data::stringCallback( const std_msgs::String::ConstPtr& msg )
{
    if( msg->data == "RED")
        ROS_INFO("RED");
    else if( msg->data == "GREEN")
        ROS_INFO("GREEN");
    else if( msg->data == "LEFT_GREEN")
        ROS_INFO("LEFT_GREEN");
    else if( msg->data == "LEFT_RED")
        ROS_INFO("LEFT_RED");
}

