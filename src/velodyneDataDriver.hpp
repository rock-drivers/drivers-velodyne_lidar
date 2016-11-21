#ifndef _VELODYNE_DATA_DRIVER_HPP_
#define _VELODYNE_DATA_DRIVER_HPP_

#include <iodrivers_base/Driver.hpp>
#include <velodyne_lidar/velodyneProtocolTypes.hpp>
#include <base/samples/DepthMap.hpp>
#include "velodyneConstants.hpp"

namespace aggregator
{
    class TimestampEstimator;
}

namespace velodyne_lidar
{

enum LaserHead
{
    LowerHead,
    UpperHead
};

class VelodyneDataDriver : public iodrivers_base::Driver 
{
    struct StampedDataPacket
    {
        base::Time time[VELODYNE_NUM_SHOTS];
        velodyne_data_packet packet;
    };

public:
    VelodyneDataDriver();
    virtual ~VelodyneDataDriver();

    /**
     * Read the next packet in the buffer.
     * @returns true on success, false if the buffer is empty.
     * @throws std::runtime_error on i/o failures.
     */
    bool readNewPacket();

    /**
     * @returns true if a full scan is complete, according to the target scan size.
     */
    bool isScanComplete();

    /**
     * Converts a full scan to a DepthMap sample.
     * Also clears all collected packets of the corresponding laser head.
     * @returns true on success, false if nothing to convert.
     */
    bool convertScanToSample(base::samples::DepthMap& sample, LaserHead head = UpperHead, bool copy_remission = true);

    /**
     * Clears all collected packets.
     */
    void clearCurrentScan();

    /**
     * Set the target scan size in radian.
     */
    void setScanSize(double angular_size);

    /**
     * Returns the target scan size in radian.
     */
    double getScanSize();

    /**
     * Returns the count of lost packets.
     */
    int64_t getPacketLostCount();

    /**
     * Returns the count of received packets.
     */
    int64_t getPacketReceivedCount();

    /**
     * Prints the packet
     */
    void print_packet(velodyne_data_packet_t &);

protected:

    /**
     * Helper method from Driver to extract the desired message.
     */
    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

    /**
     * Adds a new stamped packet to the upper or lower head list.
     */
    void addNewPacket(const StampedDataPacket& packet);

    uint64_t target_batch_size;
    uint64_t current_batch_size;
    uint16_t last_rotational_pos;
    uint32_t last_packet_internal_timestamp; // in microseconds
    int64_t packets_idx;
    int64_t packets_lost;
    int64_t expected_packet_period;
    aggregator::TimestampEstimator* timestamp_estimator;
    std::vector<StampedDataPacket> upper_head;
    std::vector<StampedDataPacket> lower_head;
};

};

#endif
