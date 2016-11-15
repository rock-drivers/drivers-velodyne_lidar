#include <iostream>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <sys/socket.h>
#include <aggregator/TimestampEstimator.hpp>
#include "velodyneDataDriver.hpp"

using namespace velodyne_lidar;

VelodyneDataDriver::VelodyneDataDriver() : Driver(VELODYNE_DATA_MSG_BUFFER_SIZE) 
{
    // Confirm that the UDP message buffer matches the structure size
    assert(sizeof(velodyne_data_packet_t) == VELODYNE_DATA_MSG_BUFFER_SIZE);
    
    current_batch_size = 0;
    target_batch_size = 36000; // 360 degree
    packets_received = 0;
    packets_lost = 0;
    upper_head.reserve(200);
    lower_head.reserve(200);
    last_rotational_pos = base::unknown<uint16_t>();
    last_packet_internal_timestamp = base::unknown<uint32_t>();
    timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromSeconds(1.0/VELODYNE_DRIVER_BROADCAST_FREQ_HZ));
    expected_packet_period = 1000000 / VELODYNE_DRIVER_BROADCAST_FREQ_HZ;
}

VelodyneDataDriver::~VelodyneDataDriver()
{
    delete timestamp_estimator;
}

bool VelodyneDataDriver::readNewPacket()
{
    if(getFileDescriptor() == INVALID_FD)
         iodrivers_base::UnixError("Invalid file descriptor!");
    try
    {
        StampedDataPacket data_packet;
        if(readPacket((uint8_t*)&data_packet.packet, VELODYNE_DATA_MSG_BUFFER_SIZE) == VELODYNE_DATA_MSG_BUFFER_SIZE)
        {
            timeval tv;
            if(ioctl(getFileDescriptor(), SIOCGSTAMP, &tv) >= 0)
            {
                base::Time receive_time = base::Time::fromSeconds(tv.tv_sec, tv.tv_usec);

                if(!base::isUnknown<uint32_t>(last_packet_internal_timestamp))
                {
                    uint32_t current_period = data_packet.packet.gps_timestamp - last_packet_internal_timestamp;
                    //check for wrap around
                    if(last_packet_internal_timestamp > data_packet.packet.gps_timestamp)
                        current_period = (std::numeric_limits<uint32_t>::max() - last_packet_internal_timestamp) + data_packet.packet.gps_timestamp;
                    int64_t new_packets = (int64_t)((double)current_period / (double)expected_packet_period + 0.5);
                    if(new_packets > 1)
                        packets_lost += new_packets - 1;
                    packets_received += new_packets;
                }
                last_packet_internal_timestamp = data_packet.packet.gps_timestamp;
                base::Time packet_timestamp = timestamp_estimator->update(receive_time, packets_received);
                base::Time time_between_shots = timestamp_estimator->getPeriod() * (1./(double)VELODYNE_NUM_SHOTS);
                for(unsigned i = 1; i <= VELODYNE_NUM_SHOTS; i++)
                    data_packet.time[i-1] = packet_timestamp - time_between_shots * (VELODYNE_NUM_SHOTS-i);

                addNewPacket(data_packet);
                return true;
            }
            throw iodrivers_base::UnixError("Failed to receive socket timestamp of the last packet passed to the user! " + std::string(strerror(errno)));
        }
    }
    catch(const iodrivers_base::TimeoutError& e)
    {
        // this is an expected case
    }
    return false;
}

bool VelodyneDataDriver::isScanComplete()
{
    return current_batch_size >= target_batch_size;
}

bool VelodyneDataDriver::convertScanToSample(base::samples::DepthMap& sample, LaserHead head, bool copy_remission)
{
    std::vector<StampedDataPacket> &packets = head == velodyne_lidar::UpperHead ? upper_head : lower_head;

    if(packets.empty())
        return false;

    // prepare sample
    sample.reset();
    unsigned horizontal_steps = packets.size() * VELODYNE_NUM_SHOTS;
    unsigned measurement_count = horizontal_steps * VELODYNE_NUM_LASERS;
    sample.distances.resize(measurement_count);
    if(copy_remission)
        sample.remissions.resize(measurement_count);
    sample.timestamps.resize(horizontal_steps);
    sample.time = packets.back().time[VELODYNE_NUM_SHOTS-1];
    sample.horizontal_size = horizontal_steps;
    sample.vertical_size = VELODYNE_NUM_LASERS;
    sample.horizontal_interval.resize(horizontal_steps);
    sample.vertical_interval.push_back(base::Angle::deg2Rad(VELODYNE_VERTICAL_START_ANGLE));
    sample.vertical_interval.push_back(base::Angle::deg2Rad(VELODYNE_VERTICAL_END_ANGLE));
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
            sample.horizontal_interval[column] = base::Angle::deg2Rad(360.0 - (double)shot.rotational_pos * 0.01);
            for(unsigned k = 0; k < VELODYNE_NUM_LASERS; k++)
            {
                const vel_laser_t &laser = shot.lasers[VELODYNE_FIRING_ORDER[k]];
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

    packets.clear();
    current_batch_size = 0;
    return true;
}

void VelodyneDataDriver::clearCurrentScan()
{
    upper_head.clear();
    lower_head.clear();
    current_batch_size = 0;
}

double VelodyneDataDriver::getScanSize()
{
    return (double)target_batch_size * 0.01;
}

void VelodyneDataDriver::setScanSize(double angular_size)
{
    target_batch_size = (uint64_t)(base::Angle::rad2Deg(std::abs(angular_size)) * 100.0);
}

int64_t VelodyneDataDriver::getPacketLostCount()
{
    return packets_lost;
}

int64_t VelodyneDataDriver::getPacketReceivedCount()
{
    return packets_received;
}

void VelodyneDataDriver::print_packet(velodyne_data_packet_t &packet) 
{
    for (int in = 0; in < 12 ; in++)
    {
        printf("Block %d, %d, %d:\n",in, packet.shots[in].lower_upper, packet.shots[in].rotational_pos);
        printf("Distance: ");
        for (int im = 0; im < 32 ; im ++)
        {
            printf("%d\t", packet.shots[in].lasers[im].distance);
        }
        printf("\nIntensity: ");
        for (int im = 0; im < 32 ; im ++)
        {
            printf("%d\t" , packet.shots[in].lasers[im].intensity);
        }
        printf("\n");
    }
    printf("Status: \n");
    
    std::cout << "Type:"<< (double)packet.status_type  << " Value: " <<(double)packet.status_value  <<" GPS Timestamp: " << packet.gps_timestamp << std::endl;
    base::Time gps_time = base::Time::fromMicroseconds(packet.gps_timestamp);
    std::cout<< "GPS Time:" << gps_time.toString() << std::endl;
    printf("\n\n\n\n");
}

int VelodyneDataDriver::extractPacket(const uint8_t* buffer, size_t buffer_size) const
{
    if(buffer_size < VELODYNE_DATA_MSG_BUFFER_SIZE)
    {
        // wait for complete packet
        return 0;
    }
    else if(buffer_size == VELODYNE_DATA_MSG_BUFFER_SIZE)
    {
        // drop first byte until it is a valid header byte
        if(buffer[0] != 0xFF)
            return -1;
        // found a valid packet
        return buffer_size;
    }
    // drop buffer
    return -buffer_size;
}

void VelodyneDataDriver::addNewPacket(const VelodyneDataDriver::StampedDataPacket& packet)
{
    if(packet.packet.shots[0].lower_upper == VELODYNE_UPPER_HEADER_BYTES)
    {
        if(!base::isUnknown<uint16_t>(last_rotational_pos))
            current_batch_size += packet.packet.shots[0].rotational_pos > last_rotational_pos ?
                                    packet.packet.shots[0].rotational_pos - last_rotational_pos :
                                    packet.packet.shots[0].rotational_pos + (35999 - last_rotational_pos);
        for(unsigned i = 1; i < VELODYNE_NUM_SHOTS; i++)
        {
            current_batch_size += packet.packet.shots[i].rotational_pos > packet.packet.shots[i-1].rotational_pos ?
                                    packet.packet.shots[i].rotational_pos - packet.packet.shots[i-1].rotational_pos :
                                    packet.packet.shots[i].rotational_pos + (35999 - packet.packet.shots[i-1].rotational_pos);
        }
        last_rotational_pos = packet.packet.shots[VELODYNE_NUM_SHOTS-1].rotational_pos;

        upper_head.push_back(packet);
    }
    else if(packet.packet.shots[0].lower_upper == VELODYNE_LOWER_HEADER_BYTES)
    {
        lower_head.push_back(packet);
    }
}