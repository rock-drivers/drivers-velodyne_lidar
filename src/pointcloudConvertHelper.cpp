#include "pointcloudConvertHelper.hpp"

#include <algorithm>
#include <math.h>
#include <stdexcept>

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
                                            unsigned int skip_n_horizontal_scans,
                                            std::vector<float>* remission_values)
{        
    points.clear();
    
    if(remission_values != NULL)
        remission_values->clear();

    // check if nothing to do
    if(laser_scan.horizontal_scans.empty())
        return;

    // give the vector a hint about the size it might be
    points.reserve(laser_scan.horizontal_scans.size() * laser_scan.horizontal_scans.front().vertical_scans.size());
    
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
        convertVerticalScan(laser_scan, *v_scan, points, transform, skip_invalid_points, remission_values);
    }
    
    if(remission_values != NULL)
        assert(points.size() == remission_values->size());
}

void ConvertHelper::convertScanToPointCloud(const MultilevelLaserScan& laser_scan, std::vector<Eigen::Vector3d> &points,
                                            const Eigen::Affine3d& transform_start, const Eigen::Affine3d& transform_end, 
                                            bool skip_invalid_points, unsigned int skip_n_horizontal_scans,
                                            std::vector<float>* remission_values)
{
    points.clear();
    
    if(remission_values != NULL)
        remission_values->clear();

    // check if nothing to do
    if(laser_scan.horizontal_scans.empty())
        return;

    // give the vector a hint about the size it might be
    points.reserve(laser_scan.horizontal_scans.size() * laser_scan.horizontal_scans.front().vertical_scans.size());

    // compute interpolated transformations
    unsigned interpolation_steps = 36;
    Eigen::Vector3d translation_delta = transform_end.translation() - transform_start.translation();
    Eigen::Quaterniond rotation_start = Eigen::Quaterniond(transform_start.linear());
    Eigen::Quaterniond rotation_end = Eigen::Quaterniond(transform_end.linear());
    base::Angle horizontal_start_angle = laser_scan.horizontal_scans.front().horizontal_angle;
    double two_pi = 2 * M_PI;
    double angle_step_size = two_pi / (double)interpolation_steps;
    std::vector<Eigen::Affine3d> transforms(interpolation_steps);
    for(unsigned i = 0; i < interpolation_steps; i++)
    {
        transforms[i] = rotation_start.slerp((double)i / (double)(interpolation_steps-1), rotation_end);
        transforms[i].pretranslate(transform_start.translation() + ((double)i / (double)(interpolation_steps-1)) * translation_delta);
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
        double linear_angle = (v_scan->horizontal_angle - horizontal_start_angle).getRad() * -1.0;
        linear_angle = linear_angle < 0 ? linear_angle + two_pi : linear_angle;
        convertVerticalScan(laser_scan, *v_scan, points, transforms[(int)(linear_angle / angle_step_size)], skip_invalid_points, remission_values);
    }
    
    if(remission_values != NULL)
        assert(points.size() == remission_values->size());
}

void ConvertHelper::convertVerticalScan(const MultilevelLaserScan& laser_scan, const MultilevelLaserScan::VerticalMultilevelScan& v_scan, std::vector<Eigen::Vector3d> &points,
                                        const Eigen::Affine3d& transform, bool skip_invalid_points, std::vector<float>* remission_values)
{
    RotationLUT* lut = RotationLUT::getInstance();
    Eigen::Quaterniond horizontal_rotation = lut->getYawRotation(v_scan.horizontal_angle.getRad());

    // convert vertical scans
    for(unsigned int i = 0; i < v_scan.vertical_scans.size(); i++)
    {
        Eigen::Vector3d point;
        if(laser_scan.isRangeValid(v_scan.vertical_scans[i].range))
        {
            //get a vector with the right length
            point = ((double)v_scan.vertical_scans[i].range / 1000.0) * Eigen::Vector3d::UnitX();
            //rotate
            point = horizontal_rotation * lut->getPitchRotation(base::Angle::fromRad(v_scan.vertical_start_angle.getRad() + i * v_scan.vertical_angular_resolution).getRad()) * point;
            
            point = transform * point;
            points.push_back(point);
            
            if(remission_values != NULL)
                remission_values->push_back(v_scan.vertical_scans[i].remission);
        }
        else if(!skip_invalid_points)
        {
            points.push_back(Eigen::Vector3d(base::unknown<double>(), base::unknown<double>(), base::unknown<double>()));
            
            if(remission_values != NULL)
                remission_values->push_back(v_scan.vertical_scans[i].remission);
        }
    }
}

struct HorizontalBin
{
    std::vector<float> remissions;
    std::vector<uint32_t> ranges;
    
    void clear() 
    {
        remissions.clear();
        ranges.clear();
    }
};

void ConvertHelper::horizontalBinning(const MultilevelLaserScan &laser_scan, MultilevelLaserScan &filtered_laser_scan, double angular_bin_size)
{
    filtered_laser_scan.horizontal_scans.clear();
    filtered_laser_scan.time = laser_scan.time;
    filtered_laser_scan.min_range = laser_scan.min_range;
    filtered_laser_scan.max_range = laser_scan.max_range;
    
    // check for invalid input or nothing to do
    if(angular_bin_size > M_PI_2 || angular_bin_size <= 0.0 || laser_scan.horizontal_scans.size() < 2 || 
       laser_scan.horizontal_scans[0].horizontal_angle < laser_scan.horizontal_scans[1].horizontal_angle ||
       laser_scan.horizontal_scans.front().vertical_scans.size() == 0)
        return;
    
    base::Angle corrected_bin_size = base::Angle::fromRad(M_PI / floor(M_PI / angular_bin_size));
    base::Angle bin_start_angle = laser_scan.horizontal_scans[0].horizontal_angle + 0.5 * (laser_scan.horizontal_scans[laser_scan.horizontal_scans.size()-1].horizontal_angle - laser_scan.horizontal_scans[0].horizontal_angle);
    base::Angle bin_angle = bin_start_angle - 0.5 * corrected_bin_size;
    base::Angle bin_end_angle = bin_start_angle - corrected_bin_size;
    
    std::vector<HorizontalBin> bin_sum(laser_scan.horizontal_scans.front().vertical_scans.size());
    for(unsigned i = 0; i < bin_sum.size(); i++)
        bin_sum[i].clear();
    
    unsigned bin_value_count = 0;
    unsigned values_handled = 0;

    std::vector<MultilevelLaserScan::VerticalMultilevelScan>::const_iterator v_scan = laser_scan.horizontal_scans.begin();
    while(v_scan < laser_scan.horizontal_scans.end() || bin_value_count > 0) 
    {        
        if(v_scan < laser_scan.horizontal_scans.end() && !(v_scan->horizontal_angle == bin_end_angle) && v_scan->horizontal_angle.isInRange(bin_end_angle, bin_start_angle))
        {
            // add values to bin
            bin_value_count += 1;
            for(unsigned i = 0; i < bin_sum.size(); i++)
            {
                if(v_scan->vertical_scans[i].range > MultilevelLaserScan::MAX_RANGE_ERROR && filtered_laser_scan.isRangeValid(v_scan->vertical_scans[i].range))
                {
                    bin_sum[i].ranges.push_back(v_scan->vertical_scans[i].range);
                    bin_sum[i].remissions.push_back(v_scan->vertical_scans[i].remission);
                }
            }
            v_scan++;
        }
        else
        {
            if(bin_value_count > 0)
            {
                values_handled += bin_value_count;
                
                // add new vertical scan
                MultilevelLaserScan::VerticalMultilevelScan new_v_scan;
                new_v_scan.time = (v_scan-1)->time;
                new_v_scan.vertical_start_angle = (v_scan-1)->vertical_start_angle;
                new_v_scan.vertical_angular_resolution = (v_scan-1)->vertical_angular_resolution;
                new_v_scan.horizontal_angle = bin_angle;
                new_v_scan.vertical_scans.resize(bin_sum.size());
                for(unsigned i = 0; i < bin_sum.size(); i++)
                {
                    if(bin_sum[i].ranges.size() >= (double)bin_value_count * 0.5)
                    {
                        if(bin_sum[i].ranges.size() > 1)
                        {
                            // compute standart deviation of the current bin
                            double sigma = 0.0;
                            double min_sigma = 1; // 1 mm
                            uint32_t range_sum = 0;
                            for(unsigned j = 0; j < bin_sum[i].ranges.size(); j++)
                            {
                                range_sum += bin_sum[i].ranges[j];
                            }
                            double mu = (double)range_sum / (double)bin_sum[i].ranges.size();
                            
                            for(unsigned j = 0; j < bin_sum[i].ranges.size(); j++)
                            {
                                sigma += pow((double)bin_sum[i].ranges[j] - mu, 2);
                            }
                            sigma = sqrt(sigma / (double)bin_sum[i].ranges.size());
                            if(sigma < min_sigma)
                                sigma = min_sigma;
                            
                            // compute the average value of all values in the range of the standart deviation
                            range_sum = 0;
                            float remission_sum = 0.0;
                            unsigned inlier_count = 0;
                            for(unsigned j = 0; j < bin_sum[i].ranges.size(); j++)
                            {
                                if((bin_sum[i].ranges[j] > mu - sigma) && (bin_sum[i].ranges[j] < mu + sigma))
                                {
                                    range_sum += bin_sum[i].ranges[j];
                                    remission_sum += bin_sum[i].remissions[j];
                                    inlier_count++;
                                }
                            }
                            
                            if(inlier_count > 0)
                            {
                                new_v_scan.vertical_scans[i].range = range_sum / inlier_count;
                                new_v_scan.vertical_scans[i].remission = remission_sum / (float)inlier_count;
                            }
                            else
                            {
                                new_v_scan.vertical_scans[i].range = MultilevelLaserScan::TOO_FAR;
                                new_v_scan.vertical_scans[i].remission = 0.0f;
                            }
                            
                        }
                        else
                        {
                            new_v_scan.vertical_scans[i].range = bin_sum[i].ranges.front();
                            new_v_scan.vertical_scans[i].remission = bin_sum[i].remissions.front();
                        }
                    }
                    else
                    {
                        new_v_scan.vertical_scans[i].range = MultilevelLaserScan::TOO_FAR;
                        new_v_scan.vertical_scans[i].remission = 0.0f;
                    }
                }
                filtered_laser_scan.horizontal_scans.push_back(new_v_scan);
                
                // reset values
                bin_value_count = 0;
                for(unsigned i = 0; i < bin_sum.size(); i++)
                    bin_sum[i].clear();
            }
            
            // move bin
            if(v_scan < laser_scan.horizontal_scans.end())
            {
                bin_start_angle = bin_end_angle;
                bin_angle = bin_start_angle - 0.5 * corrected_bin_size;
                bin_end_angle = bin_start_angle - corrected_bin_size;
            }
        }
    }
    
    assert(values_handled == laser_scan.horizontal_scans.size());
}

void ConvertHelper::filterOutliers(const MultilevelLaserScan& laser_scan, MultilevelLaserScan& filtered_laser_scan, double max_deviation_angle, unsigned min_neighbors)
{
    // copy data
    filtered_laser_scan.horizontal_scans = laser_scan.horizontal_scans;
    filtered_laser_scan.time = laser_scan.time;
    filtered_laser_scan.min_range = laser_scan.min_range;
    filtered_laser_scan.max_range = laser_scan.max_range;
    
    // this should always be the case
    if(filtered_laser_scan.min_range <= MultilevelLaserScan::MAX_RANGE_ERROR) 
        filtered_laser_scan.min_range = MultilevelLaserScan::MAX_RANGE_ERROR+1;
    
    // check for invalid input or nothing to do
    if(max_deviation_angle >= M_PI || max_deviation_angle < 0.0 || laser_scan.horizontal_scans.size() == 0 || 
       laser_scan.horizontal_scans.front().vertical_scans.size() == 0)
        return;
    
    for(unsigned i = 0; i < laser_scan.horizontal_scans.size(); i++)
    {
        const MultilevelLaserScan::VerticalMultilevelScan* left_scan = &laser_scan.horizontal_scans[i == 0 ? laser_scan.horizontal_scans.size()-1 : i-1];
        const MultilevelLaserScan::VerticalMultilevelScan* current_scan = &laser_scan.horizontal_scans[i];
        const MultilevelLaserScan::VerticalMultilevelScan* right_scan = &laser_scan.horizontal_scans[i == laser_scan.horizontal_scans.size()-1 ? 0 : i+1];
        double left_h_angle = std::abs((current_scan->horizontal_angle - left_scan->horizontal_angle).getRad());
        double right_h_angle = std::abs((right_scan->horizontal_angle - current_scan->horizontal_angle).getRad());
        for(unsigned j = 0; j < laser_scan.horizontal_scans[i].vertical_scans.size(); j++)
        {
            // continue if current ray is already invalid
            if(!filtered_laser_scan.isRangeValid(current_scan->vertical_scans[j].range))
                continue;
            
            double current_ray_dist = current_scan->vertical_scans[j].range * 0.001;
            unsigned neighbors_found = 0;
            
            // find at least one valid neighbor
            double v_angle = laser_scan.horizontal_scans[i].vertical_angular_resolution;
            if(filtered_laser_scan.isRangeValid(left_scan->vertical_scans[j].range) && 
                    max_deviation_angle >= computeMaximumAngle(left_h_angle, current_ray_dist, left_scan->vertical_scans[j].range * 0.001))
            {
                neighbors_found++;
                if(neighbors_found >= min_neighbors)
                    continue;
            }
            if(filtered_laser_scan.isRangeValid(right_scan->vertical_scans[j].range) && 
                    max_deviation_angle >= computeMaximumAngle(right_h_angle, current_ray_dist, right_scan->vertical_scans[j].range * 0.001))
            {
                neighbors_found++;
                if(neighbors_found >= min_neighbors)
                    continue;
            }
            if(j > 0 && filtered_laser_scan.isRangeValid(current_scan->vertical_scans[j-1].range) &&
                    max_deviation_angle >= computeMaximumAngle(v_angle, current_ray_dist, current_scan->vertical_scans[j-1].range * 0.001))
            {
                neighbors_found++;
                if(neighbors_found >= min_neighbors)
                    continue;
            }
            if(j < current_scan->vertical_scans.size()-1 && filtered_laser_scan.isRangeValid(current_scan->vertical_scans[j+1].range) &&
                    max_deviation_angle >= computeMaximumAngle(v_angle, current_ray_dist, current_scan->vertical_scans[j+1].range * 0.001))
            {
                neighbors_found++;
            }
            
            if(neighbors_found >= min_neighbors)
                continue;

            // invalidate the current ray if too less valid neighbors have been found
            filtered_laser_scan.horizontal_scans[i].vertical_scans[j].range = MultilevelLaserScan::MEASUREMENT_ERROR;
        }
    }
}


double ConvertHelper::computeMaximumAngle(double angle_between_rays, double dist_ray_1, double dist_ray_2)
{
    if(dist_ray_1 <= 0.0 || dist_ray_2 <= 0.0)
        throw std::range_error("The ray distants have to be positive!");
    else if(angle_between_rays <= 0.0 || angle_between_rays >= M_PI)
        throw std::range_error("The angle between the rays has to be positive and smaller than PI");
    else if(dist_ray_1 == dist_ray_2)
        return (M_PI - angle_between_rays) * 0.5;
    
    double min_dist = std::min(dist_ray_1, dist_ray_2);
    double max_dist = std::max(dist_ray_1, dist_ray_2);
    
    double oposite_angle = atan2(sin(angle_between_rays) * min_dist, max_dist - (cos(angle_between_rays) * min_dist));
    return M_PI - (oposite_angle + angle_between_rays);
}

void ConvertHelper::verticalClipping(const MultilevelLaserScan &laser_scan, MultilevelLaserScan &filtered_laser_scan, const base::Angle &start_angle, const base::Angle &end_angle)
{
    if(start_angle.getRad() >= end_angle.getRad())
        throw std::range_error("The start angle has to be smaller then the end angle.");

    // copy data
    filtered_laser_scan.time = laser_scan.time;
    filtered_laser_scan.min_range = laser_scan.min_range;
    filtered_laser_scan.max_range = laser_scan.max_range;
    filtered_laser_scan.horizontal_scans.clear();

    // check for invalid input or nothing to do
    if(laser_scan.horizontal_scans.size() == 0 || laser_scan.horizontal_scans.front().vertical_scans.size() == 0 ||
        (start_angle.getRad() <= laser_scan.horizontal_scans.front().vertical_start_angle.getRad() && end_angle.getRad() >= 
        laser_scan.horizontal_scans.front().vertical_start_angle.getRad() +    
        (double)laser_scan.horizontal_scans.front().vertical_scans.size() * laser_scan.horizontal_scans.front().vertical_angular_resolution))
        return;

    // find start and end index
    const MultilevelLaserScan::VerticalMultilevelScan &first_v_scan = laser_scan.horizontal_scans.front();
    int start_index = -1;
    base::Angle new_v_start_angle;
    int end_index = -1;
    for(unsigned i = 0; i < first_v_scan.vertical_scans.size(); i++)
    {
        if(start_angle.getRad() <= first_v_scan.vertical_start_angle.getRad() + (first_v_scan.vertical_angular_resolution * (double)i))
        {
            new_v_start_angle = base::Angle::fromRad(first_v_scan.vertical_start_angle.getRad() + (first_v_scan.vertical_angular_resolution * (double)i));
            start_index = i;
            break;
        }
    }
    for(unsigned i = 0; i < first_v_scan.vertical_scans.size(); i++)
    {
        if(end_angle.getRad() >= first_v_scan.vertical_start_angle.getRad() + (first_v_scan.vertical_angular_resolution * (double)i))
        {
            end_index = i;
        }
        else
            break;
    }

    if(end_index < 0 || start_index < 0 || start_index > end_index)
        throw std::range_error("clipping range is out of laser scanner resolution.");

    filtered_laser_scan.horizontal_scans.resize(laser_scan.horizontal_scans.size());
    for(unsigned i = 0; i < laser_scan.horizontal_scans.size(); i++)
    {
        filtered_laser_scan.horizontal_scans[i].horizontal_angle = laser_scan.horizontal_scans[i].horizontal_angle;
        filtered_laser_scan.horizontal_scans[i].vertical_scans.reserve((end_index - start_index) + 1);
        filtered_laser_scan.horizontal_scans[i].vertical_start_angle = new_v_start_angle;
        filtered_laser_scan.horizontal_scans[i].vertical_angular_resolution = laser_scan.horizontal_scans[i].vertical_angular_resolution;
        filtered_laser_scan.horizontal_scans[i].time = laser_scan.horizontal_scans[i].time;
        for(unsigned j = start_index; j <= end_index; j++)
        {
            filtered_laser_scan.horizontal_scans[i].vertical_scans.push_back(laser_scan.horizontal_scans[i].vertical_scans[j]);
        }
    }
}
