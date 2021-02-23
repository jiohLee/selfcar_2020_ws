#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_traits.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/pcl_base.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;


class Clustering
{
public:
    VPointCloud Euclidean_Clustering(VPointCloud point, double Range, int min, int max);
    visualization_msgs::MarkerArray marker_result;
    void makeBoundingBox(std::vector<std::vector<cv::Point2f> > arr);

    //VPointCloud MakeCenterPoint(VPointCloud point);
private:
    std::vector<std::vector<cv::Point2f>> box_config_pt_arr;

};
