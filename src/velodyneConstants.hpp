#ifndef _VELODYNE_CONSTANTS_HPP_
#define _VELODYNE_CONSTANTS_HPP_

#include <vector>


namespace velodyne_lidar
{

    static const unsigned int VELODYNE_NUM_SHOTS = 12; // The number of shots per packet
    static const unsigned int VELODYNE_NUM_LASER_CHANNELS = 32; // The number of lasers per shot
    static const unsigned int VELODYNE_ORIENTATION_READINGS = 3; // The number of orientation readings
    static const unsigned int MIN_SENSING_DISTANCE = 500; //1m in 2mm units
    static const unsigned int MAX_SENSING_DISTANCE = 35000; //70m in 2mm units
    static const unsigned int VELODYNE_DATA_MSG_BUFFER_SIZE = 1206; //The size of a data packet
    static const unsigned int VELODYNE_POSITIONING_MSG_BUFFER_SIZE = 512; //The size of a positioning packet

    static const unsigned int VELODYNE_DATA_UDP_PORT = 2368; //The port the Velodyne sends laser data to
    static const unsigned int VELODYNE_POSITIONING_UDP_PORT = 8308; //The port the Velodyne sends positioning data to

    /**
     * Return mode of the light signal
     */
    enum ReturnMode
    {
        StrongestReturn = 0x37,
        LastReturn = 0x38,
        DualReturn = 0x39
    };

    /**
     * Sensor type
     */
    enum SensorType
    {
        VELODYNE_HDL64E = 0x20,
        VELODYNE_HDL32E = 0x21,
        VELODYNE_VLP16 = 0x22
    };
};

#endif
