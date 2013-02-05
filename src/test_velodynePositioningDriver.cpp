#include "velodynePositioningDriver.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    velodyne_lidar::VelodynePositioningDriver velodyne_driver;

    velodyne_driver.openUDP("", velodyne_lidar::VELODYNE_POSITIONING_UDP_PORT);
    
    std::cerr << "velodyne positioning driver starting active loop..." << std::endl;
    velodyne_lidar::velodyne_positioning_packet_t buffer;
    while (true)
    {
        velodyne_driver.readPacket((uint8_t*)&buffer, velodyne_lidar::VELODYNE_POSITIONING_MSG_BUFFER_SIZE, 100000, 100000);
        velodyne_driver.print_packet(buffer);
    }
    return 0; 
}