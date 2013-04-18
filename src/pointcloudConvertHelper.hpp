#ifndef _VELODYNE_LIDAR_POINTCLOUD_CONVERT_HELPER_HPP_
#define _VELODYNE_LIDAR_POINTCLOUD_CONVERT_HELPER_HPP_

#include <vector>
#include <velodyne_lidar/MultilevelLaserScan.h>

namespace velodyne_lidar
{
    
class ConvertHelper
{    
public:
    
    /**
     * Converts the multilevel laser scan into a point cloud according to the given transformation matrix.
     * To speed up the process invalid points and a set of horizontal scans can be skipped.
     * Skipping horizontal scans will result in loosing information. 
     * It will allways skip n horizontal scans and than convert one and so on.
    */
    static void convertScanToPointCloud(const MultilevelLaserScan &laser_scan, std::vector<Eigen::Vector3d> &points,
                                 const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(), bool skip_invalid_points = true, 
                                 unsigned int skip_n_horizontal_scans = 0, std::vector<float>* remission_values = NULL);
    
private:
    ConvertHelper();
    ~ConvertHelper();
};
    
};

#endif