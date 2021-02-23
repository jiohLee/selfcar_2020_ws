#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>



typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

class HeightMap
{
    public:
        VPointCloud Make_HM(VPointCloud raw_point, int grid_dim_, double m_per_cell_, double height_diff_threshold_);
    private:
        VPointCloud obstacle_cloud_;
//        cv::Mat obstacle_img_;
};
