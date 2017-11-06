#include "VelodyneHDL32EDriver.hpp"

using namespace velodyne_lidar;

static const uint16_t UPPER_HEADER_BYTES = 0xEEFF; //The byte indicating a upper shot
static const uint16_t LOWER_HEADER_BYTES = 0xDDFF; //The byte indicating a lower shot
static const unsigned int NUM_LASERS = 32;
static const unsigned int FIRING_ORDER[] = {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1,
                                            30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0};
static const double DRIVER_BROADCAST_FREQ_HZ = 1808.0;
static const double VERTICAL_RESOLUTION = 1.0 + 1.0/3.0; // vertical resolution in degree
static const double VERTICAL_START_ANGLE = -VERTICAL_RESOLUTION * 8.0; // vertical start angle in degree
static const double VERTICAL_END_ANGLE = VERTICAL_RESOLUTION * 23.0; // vertical end angle in degree

VelodyneHDL32EDriver::VelodyneHDL32EDriver() : VelodyneDataDriver(velodyne_lidar::VELODYNE_HDL32E, DRIVER_BROADCAST_FREQ_HZ)
{

}

VelodyneHDL32EDriver::~VelodyneHDL32EDriver()
{

}

double VelodyneHDL32EDriver::getBroadcastFrequency()
{
    return DRIVER_BROADCAST_FREQ_HZ;
}

bool VelodyneHDL32EDriver::convertScanToSample(base::samples::DepthMap& sample, bool copy_remission)
{
    if(packets.empty())
        return false;

    if(packets[0].packet.return_mode == DualReturn)
        throw std::runtime_error("Return mode dual return is not supported!");

    // prepare sample
    sample.reset();
    unsigned horizontal_steps = packets.size() * VELODYNE_NUM_SHOTS;
    unsigned measurement_count = horizontal_steps * NUM_LASERS;
    sample.distances.resize(measurement_count);
    if(copy_remission)
        sample.remissions.resize(measurement_count);
    sample.timestamps.resize(horizontal_steps);
    sample.time = packets.back().time[VELODYNE_NUM_SHOTS-1];
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
            unsigned column = (i * VELODYNE_NUM_SHOTS) + j;
            sample.timestamps[column] = packets[i].time[j];
            const velodyne_fire& shot = packets[i].packet.shots[j];
            sample.horizontal_interval[column] = base::Angle::fromDeg(360.0 - (double)shot.rotational_pos * 0.01).getRad();
            for(unsigned k = 0; k < NUM_LASERS; k++)
            {
                const vel_laser_t &laser = shot.lasers[FIRING_ORDER[k]];
                if(laser.distance == 0) // zero means no return within max range
                    distances(k, column) = base::infinity<base::samples::DepthMap::scalar>();
                else if(laser.distance <= min_sensing_distance) // dismiss all values under the configured limit
                    distances(k, column) = 0.f;
                else
                    distances(k, column) = (float)laser.distance * 0.002f; // velodyne acquires in 2mm-units

                if(copy_remission)
                    remissions(k, column) = (float)laser.intensity / 255.0f;
            }
        }
    }

    packets.clear();
    current_batch_size = 0;
    return true;
}
