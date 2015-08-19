#ifndef _VELODYNE_LIDAR_MultilevelLaserScan_H_
#define _VELODYNE_LIDAR_MultilevelLaserScan_H_

#include <vector>
#include <boost/cstdint.hpp>
#include <base/Time.hpp>
#include <base/Angle.hpp>
#include <base/Float.hpp>

namespace velodyne_lidar
{
    
/** @deprecated This type is deprecated, please use
 *  base::samples::DepthMap instead!
 */
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
    
    MultilevelLaserScan() : time(base::Time::now()), min_range(MAX_RANGE_ERROR+1), max_range(0) {};
    
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
        if(!boost::math::isnan(range) && !boost::math::isinf(range) && range >= min_range && range <= max_range)
            return true;
        return false;
    }
};
    
};

#endif
