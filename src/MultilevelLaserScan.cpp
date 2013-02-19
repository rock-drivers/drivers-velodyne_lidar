#include "MultilevelLaserScan.h"

using namespace velodyne_lidar;

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

Eigen::Quaterniond MultilevelLaserScan::getHorizontalRotation(const base::Angle &rad) const
{
    RotationLUT* lut = RotationLUT::getInstance();
    return lut->getYawRotation(rad.getRad());
}

Eigen::Quaterniond MultilevelLaserScan::getVerticalRotation(const base::Angle &rad) const
{
    RotationLUT* lut = RotationLUT::getInstance();
    return lut->getPitchRotation(rad.getRad());
}