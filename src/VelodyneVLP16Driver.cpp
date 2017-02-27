#include "VelodyneVLP16Driver.hpp"
#include <base/Time.hpp>

using namespace velodyne_lidar;

static const uint16_t VLP16_HEADER_BYTES = 0xFFEE; //VLP-16 identification bytes
static const unsigned int NUM_LASERS = 16; // The number of lasers per shot
static const unsigned int FIRING_ORDER[] = {15, 13, 11, 9, 7, 5, 3, 1,
                                            14, 12, 10, 8, 6, 4, 2, 0};
static const double DRIVER_BROADCAST_FREQ_HZ = 754.0; //The rate of broadcast packets.
static const double VERTICAL_RESOLUTION = 2.0; // vertical resolution in degree
static const double VERTICAL_START_ANGLE = -VERTICAL_RESOLUTION * 7.5;
static const double VERTICAL_END_ANGLE = VERTICAL_RESOLUTION * 7.5; // vertical end angle in degree
static const uint64_t CYCLE_TIME = 55.296; // cycle time of one firing in micro seconds

VelodyneVLP16Driver::VelodyneVLP16Driver() : VelodyneDataDriver(velodyne_lidar::VELODYNE_VLP16, DRIVER_BROADCAST_FREQ_HZ)
{

}

VelodyneVLP16Driver::~VelodyneVLP16Driver()
{

}

double VelodyneVLP16Driver::getBroadcastFrequency()
{
    return DRIVER_BROADCAST_FREQ_HZ;
}

bool VelodyneVLP16Driver::convertScanToSample(base::samples::DepthMap& sample, bool copy_remission)
{
    if(packets.empty())
        return false;

    if(packets[0].packet.return_mode == DualReturn)
        throw std::runtime_error("Return mode dual return is not supported!");

    // prepare sample
    sample.reset();
    unsigned horizontal_steps = 2 * packets.size() * VELODYNE_NUM_SHOTS;
    unsigned measurement_count = horizontal_steps * NUM_LASERS;
    sample.distances.resize(measurement_count);
    if(copy_remission)
        sample.remissions.resize(measurement_count);
    sample.timestamps.resize(horizontal_steps);
    sample.time = packets.back().time[VELODYNE_NUM_SHOTS-1] + base::Time::fromMicroseconds(CYCLE_TIME);
    sample.horizontal_size = horizontal_steps;
    sample.vertical_size = NUM_LASERS;
    sample.horizontal_interval.resize(horizontal_steps);
    sample.vertical_interval.push_back(base::Angle::deg2Rad(VERTICAL_START_ANGLE));
    sample.vertical_interval.push_back(base::Angle::deg2Rad(VERTICAL_END_ANGLE));
    sample.horizontal_projection = base::samples::DepthMap::POLAR;
    sample.vertical_projection = base::samples::DepthMap::POLAR;
    base::samples::DepthMap::DepthMatrixMap distances = sample.getDistanceMatrixMap();
    base::samples::DepthMap::DepthMatrixMap remissions = base::samples::DepthMap::DepthMatrixMap(sample.remissions.data(), sample.vertical_size, sample.horizontal_size);

    for(unsigned i = 0; i < packets.size(); i++)
    {
        for(unsigned j = 0; j < VELODYNE_NUM_SHOTS; j++)
        {
            const velodyne_fire& shot = packets[i].packet.shots[j];
            // handle first firing
            unsigned column = (i * VELODYNE_NUM_SHOTS + j) * 2;
            sample.timestamps[column] = packets[i].time[j];
            sample.horizontal_interval[column] = base::Angle::fromDeg(360.0 - (double)shot.rotational_pos * 0.01).getRad();

            for(unsigned k = 0; k < NUM_LASERS; k++)
            {
                const vel_laser_t &laser = shot.lasers[FIRING_ORDER[k]];
                if(laser.distance == 0) // zero means no return within max range
                    distances(k, column) = base::infinity<base::samples::DepthMap::scalar>();
                else if(laser.distance <= MIN_SENSING_DISTANCE) // dismiss all values under 1m
                    distances(k, column) = 0.f;
                else
                    distances(k, column) = (float)laser.distance * 0.002f; // velodyne acquires in 2mm-units

                if(copy_remission)
                    remissions(k, column) = (float)laser.intensity / 255.0f;
            }
            // handle second firing
            column++;
            sample.horizontal_interval[column] = base::NaN<double>();
            for(unsigned k = 0; k < NUM_LASERS; k++)
            {
                const vel_laser_t &laser = shot.lasers[NUM_LASERS + FIRING_ORDER[k]];
                if(laser.distance == 0) // zero means no return within max range
                    distances(k, column) = base::infinity<base::samples::DepthMap::scalar>();
                else if(laser.distance <= MIN_SENSING_DISTANCE) // dismiss all values under 1m
                    distances(k, column) = 0.f;
                else
                    distances(k, column) = (float)laser.distance * 0.002f; // velodyne acquires in 2mm-units

                if(copy_remission)
                    remissions(k, column) = (float)laser.intensity / 255.0f;
            }
        }
    }

    // interpolate angles and timestamps for all odd entries
    base::Angle angle_step;
    for(unsigned column = 1; column < sample.horizontal_size - 1; column += 2)
    {
        base::Angle angle_before = base::Angle::fromRad(sample.horizontal_interval[column - 1]);
        base::Angle angle_after = base::Angle::fromRad(sample.horizontal_interval[column + 1]);
        angle_step = (angle_after - angle_before) * 0.5;
        sample.horizontal_interval[column] = (angle_before + angle_step).getRad();
        sample.timestamps[column] = sample.timestamps[column - 1] + (sample.timestamps[column + 1] - sample.timestamps[column - 1]) * 0.5;
    }
    sample.timestamps[sample.horizontal_size - 1] = sample.time;
    sample.horizontal_interval[sample.horizontal_size - 1] = (base::Angle::fromRad(sample.horizontal_interval[sample.horizontal_size - 2]) + angle_step).getRad();

    packets.clear();
    current_batch_size = 0;
    return true;
}