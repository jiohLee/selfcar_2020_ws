#include <clustering.h>




VPointCloud Clustering::Euclidean_Clustering(VPointCloud point, double Range, int min, int max)
 {
     VPointCloud output;

     VPointCloud::Ptr cloud_obstacle(new VPointCloud);
     cloud_obstacle = point.makeShared();

     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
     tree->setInputCloud(cloud_obstacle);
     std::vector<pcl::PointIndices> clusters;

     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
     ec.setClusterTolerance(Range);
     ec.setMinClusterSize(min);
     ec.setMaxClusterSize(max);
     ec.setSearchMethod(tree);
     ec.setInputCloud(cloud_obstacle);
     ec.extract(clusters);

     int j = 0;

     for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
     {

         // get Center Point of each Clusters
         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
         {
             pcl::PointXYZI pt = cloud_obstacle->points[*pit];
             pt.intensity = j;
             output.push_back(pt);
         }
         j++;

       }
    // makeBoundingBox(box_config_pt_arr);
    return output;
 }

//VPointCloud Clustering::MakeCenterPoint(VPointCloud point)
//{
//    VPointCloud output;
//    VPointCloud::Ptr cloud_obstacle(new VPointCloud);
//    cloud_obstacle = point.makeShared();

//    for(std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++ it)
//    {
//        pcl::CentroidPoint<VPoint> centroid;
//        VPoint center_pt;
//        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
//        {
//            center_pt = cloud_obstacle->points[*pit];
//            center_pt.intensity = 100;
//            centroid.add(center_pt);
//        }
//        centroid.get(center_pt);
//        output.push_back(center_pt);
//    }
//    return output;
//}

//void Clustering::makeBoundingBox(std::vector<std::vector<cv::Point2f> > arr)
//{
//    //marker_result.markers.clear();
//    for(int i = 0; i < arr.size(); i++)
//    {
//        visualization_msgs::Marker marker;
//        marker.header.stamp = ros::Time::now();
//        marker.header.frame_id = "velodyne";
//        marker.ns = "adaptive_clustering";
//        marker.id = i;
//        marker.type = visualization_msgs::Marker::LINE_LIST;

//        geometry_msgs::Point p[24];
//        p[0].x = arr[i][0].x;  p[0].y = arr[i][0].y;  p[0].z = -1;
//        p[1].x = arr[i][1].x;  p[1].y = arr[i][1].y;  p[1].z = -1;
//        p[2].x = arr[i][2].x;  p[2].y = arr[i][2].y;  p[2].z = -1;
//        p[3].x = arr[i][3].x;  p[3].y = arr[i][3].y;  p[3].z = -1;

//        p[4].x = arr[i][0].x;  p[4].y = arr[i][0].y;  p[4].z = 2;
//        p[5].x = arr[i][1].x;  p[5].y = arr[i][1].y;  p[5].z = 2;
//        p[6].x = arr[i][2].x;  p[6].y = arr[i][2].y;  p[6].z = 2;
//        p[7].x = arr[i][3].x;  p[7].y = arr[i][3].y;  p[7].z = 2;

//        p[8].x = arr[i][0].x;  p[8].y = arr[i][0].y;  p[8].z = -1;
//        p[9].x = arr[i][1].x;  p[9].y = arr[i][1].y;  p[9].z = -1;
//        p[10].x = arr[i][0].x; p[10].y = arr[i][0].y; p[10].z = 2;
//        p[11].x = arr[i][1].x; p[11].y = arr[i][1].y; p[11].z = 2;

//        p[12].x = arr[i][0].x; p[12].y = arr[i][0].y; p[12].z = -1;
//        p[13].x = arr[i][3].x; p[13].y = arr[i][3].y; p[13].z = -1;
//        p[14].x = arr[i][0].x; p[14].y = arr[i][0].y; p[14].z = 2;
//        p[15].x = arr[i][3].x; p[15].y = arr[i][3].y; p[15].z = 2;

//        p[16].x = arr[i][1].x; p[16].y = arr[i][1].y; p[16].z = -1;
//        p[17].x = arr[i][2].x; p[17].y = arr[i][2].y; p[17].z = -1;
//        p[18].x = arr[i][1].x; p[18].y = arr[i][1].y; p[18].z = 2;
//        p[19].x = arr[i][2].x; p[19].y = arr[i][2].y; p[19].z = 2;

//        p[20].x = arr[i][2].x; p[20].y = arr[i][2].y; p[20].z = -1;
//        p[21].x = arr[i][3].x; p[21].y = arr[i][3].y; p[21].z = -1;
//        p[22].x = arr[i][2].x; p[22].y = arr[i][2].y; p[22].z = 2;
//        p[23].x = arr[i][3].x; p[23].y = arr[i][3].y; p[23].z = 2;
//        for(int j =  0; j < 24; j++){
//            marker.points.push_back(p[j]);
//        }
//        marker.scale.x = 0.02;
//        marker.color.a = 1.0;
//        marker.color.r = 0.0;
//        marker.color.g = 1.0;
//        marker.color.b = 0.5;
////        marker.lifetime = ros::Duration(0.1);
//        marker_result.markers.push_back(marker);
//    }
//}


