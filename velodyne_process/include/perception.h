#include "heightmap.h"
#include "clustering.h"

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <time.h>

class Perception: public HeightMap, Clustering
{
protected:

    ros::NodeHandle nh;
    ros::Subscriber scan_sub;           // VELODYNE LIDAR DATA Sub
    ros::Publisher heightmap_pub;       // heightmap data Pub
    ros::Publisher obj_cluster_data_pub;    // Obj clustering data to PCL
    ros::Publisher obj_center_point_pub;    // Obj center Point
    ros::Publisher marker_array_pub;    // marker_data
    ros::Publisher img_pub;             // Boundary Rectangle Img


public:
    Perception(ros::NodeHandle nh);
    void scanCallback(const sensor_msgs::PointCloud2Ptr scan); //LiDAR Raw Data
    VPointCloud Passthrough_ob(VPointCloud point, double minx, double maxx, double miny, double maxy);
    void init(VPointCloud raw_point);

};
