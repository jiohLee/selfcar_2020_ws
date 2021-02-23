#include <heightmap.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))


VPointCloud HeightMap::Make_HM(VPointCloud raw_data_, int grid_dim_, double m_per_cell_, double height_diff_threshold_)
{
    // obstacle_img_ = cv::Mat::zeros(grid_dim_, grid_dim_, CV_8UC1);

     // pass along original time stamp and frame ID
     // 1. obstacle_cloud_
     obstacle_cloud_.header.stamp    = raw_data_.header.stamp;
     obstacle_cloud_.header.frame_id = raw_data_.header.frame_id;

     size_t npoints = raw_data_.points.size();
     obstacle_cloud_.points.resize(npoints);

     size_t obs_count=0;

     float min[grid_dim_][grid_dim_];
     float max[grid_dim_][grid_dim_];
     unsigned int num[grid_dim_][grid_dim_];
     unsigned char type[grid_dim_][grid_dim_];
     bool init[grid_dim_][grid_dim_];

     for (int x = 0; x < grid_dim_; x++)
     {
         for (int y = 0; y < grid_dim_; y++)
         {
             init[x][y]=false;
             num[x][y] = 0;
             type[x][y] = 0;

         }
     }

     // build height map
     for(unsigned i = 0; i < npoints; ++i)
     {
         int x = ((grid_dim_/2) + raw_data_.points[i].x/m_per_cell_);
         int y = ((grid_dim_/2) + raw_data_.points[i].y/m_per_cell_);

         if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_)
         {
             num[x][y] += 1;
             if(!init[x][y])
             {
                 min[x][y] = raw_data_.points[i].z;
                 max[x][y] = raw_data_.points[i].z;
                 init[x][y] = true;
             }
             else
             {
                 min[x][y] = MIN(min[x][y], raw_data_.points[i].z);
                 max[x][y] = MAX(max[x][y], raw_data_.points[i].z);
             }
         }
     }

     // calculate number of obstacles, clear and unknown in each cell
     for(int x = 0; x < grid_dim_; x++)
     {
         for(int y = 0; y < grid_dim_; y++)
         {
             if(num[x][y] >= 1 )
             {
                 if (max[x][y] - min[x][y] > height_diff_threshold_)
                 {
                     type[x][y] = 2;
                 }
                 else
                 {
                     type[x][y] = 1;
                 }

             }
         }
     }


     // create clouds from grid
     double grid_offset = grid_dim_/2*m_per_cell_;
     for (int x = 0; x < grid_dim_; x++)
     {
         for (int y = 0; y < grid_dim_; y++)
         {
             if(type[x][y] == 2)     //obstacle
             {
                 obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
                 obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
                 obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
                 obs_count ++;
             }
         }
     }
     obstacle_cloud_.points.resize(obs_count);

//    for ( int x = 0; x < grid_dim_; x++)
//    {
//        for(int y = 0; y < grid_dim_; y++)
//        {
//            if(type[x][y] == 2)  // obstacle
//            {
//                int v = 250 - y*5;
//                int w = 250 - x*5;
//                obstacle_img_.at<uchar>(v, w) = 255;

//            }
//        }
//    }
//    cv::imshow("result", obstacle_img_);
//    cv::waitKey(0);

    //return obstacle_img_;
    return obstacle_cloud_;
}

