#include "pointcloudConvertHelper.hpp"

using namespace velodyne_lidar;

/**
 * provides the look up tables for the horizontal and vertical rotation.
 * needs about 2,3 mb of memory.
 */
class RotationLUT
{
public:
    static RotationLUT* getInstance()
    {
        static CGuard g;
        if( instance == 0 )
            instance = new RotationLUT();
        return instance;
    }
    
    Eigen::Quaterniond getYawRotation(double rad)
    {
        uint16_t deg = (uint16_t)((rad < 0 ? rad + 2*M_PI: rad) * rad2deg);
        return yaw[deg];
    };
    
    Eigen::Quaterniond getPitchRotation(double rad)
    {
        uint16_t deg = (uint16_t)((rad < 0 ? rad + 2*M_PI: rad) * rad2deg);
        return pitch[deg];
    };
    
private:
    RotationLUT() : rad2deg(36000.0 / (2*M_PI)), deg2rad((2*M_PI) / 36000.0)
    {
        yaw.resize(36000);
        pitch.resize(36000);
        for(unsigned i = 0; i < 36000; i++)
          yaw[i] = Eigen::Quaterniond(Eigen::AngleAxisd((double)i * deg2rad, Eigen::Vector3d::UnitZ()));
        for(unsigned i = 0; i < 36000; i++)
          pitch[i] = Eigen::Quaterniond(Eigen::AngleAxisd((double)i * deg2rad, Eigen::Vector3d::UnitY()));
    }
    RotationLUT(const RotationLUT&);
    RotationLUT operator=(const RotationLUT&);
    ~RotationLUT() {}
    
    static RotationLUT* instance;
    double rad2deg;
    double deg2rad;
    std::vector<Eigen::Quaterniond> yaw;
    std::vector<Eigen::Quaterniond> pitch;
    
    class CGuard
    {
    public:
        ~CGuard()
        {
            if( NULL != RotationLUT::instance )
            {
                delete RotationLUT::instance;
                RotationLUT::instance = NULL;
            }
        }
    };
    friend class CGuard;
};

RotationLUT*  RotationLUT::instance = 0;



void ConvertHelper::convertScanToPointCloud(const MultilevelLaserScan& laser_scan, std::vector<Eigen::Vector3d> &points,
                                            const Eigen::Affine3d& transform, bool skip_invalid_points,
                                            unsigned int skip_n_horizontal_scans)
{        
    points.clear();
    RotationLUT* lut = RotationLUT::getInstance();

    //give the vector a hint about the size it might be
    if(!laser_scan.horizontal_scans.empty())
    {
        points.reserve(laser_scan.horizontal_scans.size() * laser_scan.horizontal_scans.front().vertical_scans.size());
    }
    
    unsigned int h_skip_count = 0;
    for(std::vector<MultilevelLaserScan::VerticalMultilevelScan>::const_iterator v_scan = laser_scan.horizontal_scans.begin(); v_scan < laser_scan.horizontal_scans.end(); v_scan++) 
    {
        // check if horizontal scan should be skipped
        if(h_skip_count < skip_n_horizontal_scans)
        {
            h_skip_count++;
            continue;
        }
        h_skip_count = 0;
        
        // convert vertical scans
        for(unsigned int i = 0; i < v_scan->vertical_scans.size(); i++)
        {
            Eigen::Vector3d point;
            if(laser_scan.isRangeValid(v_scan->vertical_scans[i].range))
            {
                //get a vector with the right length
                point = ((double)v_scan->vertical_scans[i].range / 1000.0) * Eigen::Vector3d::UnitX();
                //rotate
                point = lut->getYawRotation(v_scan->horizontal_angle.getRad()) * lut->getPitchRotation(base::Angle::fromRad(v_scan->vertical_start_angle.getRad() + i * v_scan->vertical_angular_resolution).getRad()) * point;
                
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