#ifndef _VELODYNE_DATA_DRIVER_HPP_
#define _VELODYNE_DATA_DRIVER_HPP_

#include <iodrivers_base/Driver.hpp>
#include <velodyne_lidar/velodyneProtocolTypes.hpp>
#include <velodyne_lidar/MultilevelLaserScan.h>

namespace velodyne_lidar
{

class VelodyneDataDriver : public iodrivers_base::Driver 
{
public:    
    VelodyneDataDriver();
    
    /**
     * Converts the internal type velodyne_fire_t to a VerticalMultilevelScan
     */
    void convertToVerticalMultilevelScan(const velodyne_fire_t &velodyne_fire, MultilevelLaserScan::VerticalMultilevelScan &vertical_scan);
    
    /**
     * Converts the internal type vel_laser_t to a SingleScan
     */
    void convertToSingleScan(const vel_laser_t &velodyne_laser, MultilevelLaserScan::SingleScan &single_scan);

    /**
     * prints the packet
     */
    void print_packet(velodyne_data_packet_t &);

    /**
    * Helper method from Driver to extract the desired message.
    */
    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

};

};

#endif
