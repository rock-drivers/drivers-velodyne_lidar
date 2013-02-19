#ifndef _VELODYNE_LIDAR_MultilevelLaserScan_H_
#define _VELODYNE_LIDAR_MultilevelLaserScan_H_

#include <vector>
#include <boost/cstdint.hpp>
#include <base/time.h>
#include <base/angle.h>
#include <base/float.h>

namespace velodyne_lidar
{
    
struct MultilevelLaserScan
{    
   /** Special values for the ranges. If a range has one of these values, then
    * it is not valid and the value declares what is going on */
    enum LASER_RANGE_ERRORS {
        TOO_FAR            = 1, // too far
        TOO_NEAR           = 2,
        MEASUREMENT_ERROR  = 3,
        OTHER_RANGE_ERRORS = 4,
        MAX_RANGE_ERROR    = 5  
    };
    
    struct SingleScan
    {
        SingleScan() : range(TOO_FAR), remission(0.0f)  {};
        
        /** The ranges themselves: the distance to obstacles in millimeters
         */
        boost::uint32_t range;
        
        /** The remission value from the laserscan.
         * This value is not normalised and depends on various factors, like distance, 
         * angle of incidence and reflectivity of object.
         */
        float remission;
    };
    
    struct VerticalMultilevelScan
    {
        VerticalMultilevelScan() : time(base::Time::now()), horizontal_angle(base::Angle::fromRad(0.0)), vertical_start_angle(base::Angle::fromRad(0.0)), vertical_angular_resolution(0.0) {};
        
        /** The timestamp of this reading. The timestamp is the time at which one
         * vertical scan was captured.
         */
        base::Time time;
        
        /** All vertical scans on the actual horizontal angle.
         */
        std::vector<SingleScan> vertical_scans;
        
        base::Angle horizontal_angle;
        
        /** The angle at which the range readings start. Zero is at the front of
         * the device and turns upwards. 
         * This value is in radians
         */
        base::Angle vertical_start_angle;

        /** Angle difference between two scan point in radians.
         */
        double vertical_angular_resolution;
    };
    
    MultilevelLaserScan() : time(base::Time::now()), min_range(0), max_range(0) {};
    
    /** The timestamp of this reading.
     */
    base::Time time;
    
    /** All horizontal scans.
     */
    std::vector< VerticalMultilevelScan > horizontal_scans;
    
    /** minimal valid range returned by laserscanner */
    boost::uint32_t min_range;

    /** maximal valid range returned by laserscanner */
    boost::uint32_t max_range;
    
    inline bool isRangeValid(boost::uint32_t range) const
    {
        if(range >= min_range && range <= max_range)
            return true;
        return false;
    }
    
    /** converts the multilevel laser scan into a point cloud according to the given transformation matrix,
    *  the start_angle and the angular_resolution. If the transformation matrix is set to 
    *  identity the laser scan is converted into the coordinate system of the sensor (x-axis = forward,
    *  y-axis = to the left, z-axis = upwards)
    *  If a scan point is outside of valid range all its coordinates are set to NaN.
    *  Unfortunately invalid scan points can not be skipped because this would invalidate the remission association
    */
    template<typename T>
    void convertScanToPointCloud(std::vector<T> &points,
                                 const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(), 
                                 bool skip_invalid_points = true) const
    {        
        points.clear();

        //give the vector a hint about the size it might be
        if(!horizontal_scans.empty())
        {
            points.reserve(horizontal_scans.size() * horizontal_scans.front().vertical_scans.size());
        }
        
        for(std::vector<VerticalMultilevelScan>::const_iterator v_scan = horizontal_scans.begin(); v_scan < horizontal_scans.end(); v_scan++) 
        {
            for(unsigned int i = 0; i < v_scan->vertical_scans.size(); i++)
            {
                Eigen::Vector3d point;
                if(isRangeValid(v_scan->vertical_scans[i].range))
                {
                    //get a vector with the right length
                    point = ((double)v_scan->vertical_scans[i].range / 1000.0) * Eigen::Vector3d::UnitX();
                    //rotate
                    point = getHorizontalRotation(v_scan->horizontal_angle) * getVerticalRotation(base::Angle::fromRad(v_scan->vertical_start_angle.getRad() + i * v_scan->vertical_angular_resolution)) * point;
                    
                    point = transform * point;
                    points.push_back(point);
                }
                else if(!skip_invalid_points)
                {
                    points.push_back(Eigen::Vector3d(base::unknown<double>(), base::unknown<double>(), base::unknown<double>()));
                }
            }
        }
    }

    /** gets the horizontal rotation from a LUT
     */
    Eigen::Quaterniond getHorizontalRotation(const base::Angle &rad) const;
    
    /** gets the vertical rotation from a LUT
     */
    Eigen::Quaterniond getVerticalRotation(const base::Angle &rad) const;

};
    
};

#endif