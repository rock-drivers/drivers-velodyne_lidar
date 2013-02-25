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
                                 const Eigen::Affine3d& transform, bool skip_invalid_points, unsigned int skip_n_horizontal_scans);
    
    
    static void convertScanToPointCloud(const MultilevelLaserScan &laser_scan, std::vector<Eigen::Vector3d> &points)
    {
        convertScanToPointCloud(laser_scan, points, Eigen::Affine3d::Identity());
    }
    
    static void convertScanToPointCloud(const MultilevelLaserScan &laser_scan, std::vector<Eigen::Vector3d> &points,
                                        const Eigen::Affine3d& transform)
    {
        convertScanToPointCloud(laser_scan, points, transform, true);
    }
    
    static void convertScanToPointCloud(const MultilevelLaserScan &laser_scan, std::vector<Eigen::Vector3d> &points,
                                const Eigen::Affine3d& transform, bool skip_invalid_points)
    {
        convertScanToPointCloud(laser_scan, points, transform, skip_invalid_points, 0);
    }
    
private:
    ConvertHelper();
    ~ConvertHelper();
};
    
};

#endif