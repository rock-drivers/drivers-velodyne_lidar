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

VelodyneDataDriver::VelodyneDataDriver(SensorType sensor_type, double broadcast_frequency) : Driver(VELODYNE_DATA_MSG_BUFFER_SIZE), sensor_type(sensor_type)
{
    // Confirm that the UDP message buffer matches the structure size
    assert(sizeof(velodyne_data_packet_t) == VELODYNE_DATA_MSG_BUFFER_SIZE);
    
    current_batch_size = 0;
    target_batch_size = 36000; // 360 degree
    packets_idx = 0;
    packets_lost = 0;
    packets.reserve(200);
    last_rotational_pos = base::unknown<uint16_t>();
    last_packet_internal_timestamp = base::unknown<uint32_t>();
    timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromSeconds(1.0/broadcast_frequency));
    expected_packet_period = 1000000 / broadcast_frequency;
    min_sensing_distance = MIN_SENSING_DISTANCE;
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
                    packets_idx += new_packets;
                }
                last_packet_internal_timestamp = data_packet.packet.gps_timestamp;
                base::Time packet_timestamp = timestamp_estimator->update(receive_time, packets_idx);
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

void VelodyneDataDriver::clearCurrentScan()
{
    packets.clear();
    current_batch_size = 0;
}

double VelodyneDataDriver::getScanSize()
{
    return base::Angle::deg2Rad((double)target_batch_size * 0.01);
}

void VelodyneDataDriver::setScanSize(double angular_size)
{
    target_batch_size = (uint64_t)(base::Angle::rad2Deg(std::abs(angular_size)) * 100.0);
}

void VelodyneDataDriver::setMinSensingDistance(double min_distance)
{
    this->min_sensing_distance = min_distance / 0.002;
}

double VelodyneDataDriver::getMinSensingDistance()
{
    return (double)min_sensing_distance * 0.002;
}

int64_t VelodyneDataDriver::getPacketLostCount()
{
    return packets_lost;
}

int64_t VelodyneDataDriver::getPacketReceivedCount()
{
    return packets_idx - packets_lost;
}

void VelodyneDataDriver::print_packet(velodyne_data_packet_t &packet) 
{
    for (int in = 0; in < 12 ; in++)
    {
        printf("Block %d, %d, %d:\n",in, packet.shots[in].laser_header, packet.shots[in].rotational_pos);
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
    
    std::cout << "Mode:"<< (double)packet.return_mode  << " Type: " <<(double)packet.sensor_type  <<" GPS Timestamp: " << packet.gps_timestamp << std::endl;
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

        // drop samples from a different sensor head
        if(buffer[VELODYNE_DATA_MSG_BUFFER_SIZE-1] != sensor_type)
        {
            std::cerr << "Received data from a different sensor head!" << std::endl;
            return -buffer_size;
        }

        // found a valid packet
        return buffer_size;
    }
    // drop buffer
    return -buffer_size;
}

void VelodyneDataDriver::addNewPacket(const VelodyneDataDriver::StampedDataPacket& packet)
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

    packets.push_back(packet);

}
