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
    
    /**
     * Provides a binning of the horizontal scans of every vertical plane. The scans inside a bin are filtered 
     * related to the standard deviation. The average of the filtered scans is used to represent the new scan.
     * @param laser_scan input scan
     * @param filtered_laser_scan output scan
     * @param angular_bin_size is the horizontal bin size in radiant
     */
    static void horizontalBinning(const MultilevelLaserScan &laser_scan, MultilevelLaserScan &filtered_laser_scan, double angular_bin_size);
    
private:
    ConvertHelper();
    ~ConvertHelper();
};
    
};

#endif