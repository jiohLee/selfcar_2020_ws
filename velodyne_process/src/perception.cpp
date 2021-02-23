#include "perception.h"

Perception::Perception(ros::NodeHandle nh)
{
    scan_sub             = nh.subscribe<sensor_msgs::PointCloud2Ptr> ("/velodyne_points", 100, &Perception::scanCallback, this); //Subscribe Pandar 40M
    heightmap_pub        = nh.advertise<sensor_msgs::PointCloud2> ("/velo/heightmap",100, false);
   //obj_cluster_data_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velo/clusters", 100, false);
   //obj_center_point_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velo/center_point", 100, false);
   // img_pub              = nh.advertise<sensor_msgs::Image> ("/velo/height_map_img",100);
    //marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/lidar/bounding_box", 100);





}

void Perception::scanCallback(const sensor_msgs::PointCloud2Ptr scan)
{
    VPointCloud LiDAR_Point;
    pcl::fromROSMsg(*scan, LiDAR_Point);
    init(LiDAR_Point);
}

VPointCloud Perception::Passthrough_ob(VPointCloud point, double minx, double maxx, double miny, double maxy) //장애물의 전체적인 범위 설정을 통해서 필요없는 부분 제거
{
    VPointCloud::Ptr cloud (new VPointCloud); VPointCloud::Ptr cloud_filter (new VPointCloud);
    VPointCloud filter;
    pcl::PassThrough <VPoint> pass;

    *cloud = point;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5, 1.5);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);   
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minx, maxx);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(miny, maxy);
    pass.filter(*cloud_filter);
    filter = *cloud_filter;

    return filter;
}

void Perception::init(VPointCloud raw_point)
{

    //time lap start
    ros::Time start = ros::Time::now();

    // Velodyne Point Cloud process
    VPointCloud roi_point = Passthrough_ob(raw_point, 0, 15, -6, 6);


    //cv::Mat hMapMat = Make_HM(roi_point, 500, 0.1, 0.01);


    VPointCloud hMap = Make_HM(roi_point, 500, 0.1, 0.05);
  //  VPointCloud cPoint = Euclidean_Clustering(hMap, 0.5, 5, 300);



    // time lap end
    ros::Time end = ros::Time::now();
    double result = (end - start).toSec();
    std::cout << result  *1000.0 << "ms" <<std::endl;


    /* output1 : Height map data*/
    sensor_msgs::PointCloud2 hmap_output;
    pcl::toROSMsg(hMap, hmap_output);
    hmap_output.header.frame_id = "velodyne";
    heightmap_pub.publish(hmap_output);


    /* output2 : clustered data (Euclidean Clustering)
       convert Ros Msg : VPointCloud -> PointCloud2 */
  //  sensor_msgs::PointCloud2 cluster_output;
   // pcl::toROSMsg(cPoint, cluster_output);
    /* convert PointCloud2 -> PointCloud*/
//     sensor_msgs::PointCloud output_arr;
//     sensor_msgs::convertPointCloud2ToPointCloud(output, output_arr);
//    cluster_output.header.frame_id = "velodyne";
//    obj_cluster_data_pub.publish(cluster_output);





    //marker_array_pub.publish(marker_result);
}


