#ifndef _VELODYNE_PROTOCOL_TYPES_HPP_
#define _VELODYNE_PROTOCOL_TYPES_HPP_

#include <velodyne_lidar/velodyneConstants.hpp>
#include <cstring>

namespace velodyne_lidar
{
    // Velodyne datastructures
    typedef struct vel_laser {
        uint16_t distance; // 2mm increments
        uint8_t intensity; // 255 being brightest
        
    } __attribute__((packed)) vel_laser_t;

    typedef struct velodyne_fire {
        uint16_t laser_header;
        uint16_t rotational_pos; // 0-35999  divide by 100 for degrees
        vel_laser_t lasers[VELODYNE_NUM_LASER_CHANNELS]; 
    } __attribute__((packed)) velodyne_fire_t ;

    typedef struct velodyne_data_packet {
        velodyne_fire_t shots[VELODYNE_NUM_SHOTS];  
        uint32_t gps_timestamp; // in microseconds from the top of the hour
        uint8_t return_mode;
        uint8_t sensor_type;
        
        velodyne_data_packet()
        {
            //clear everything 
            memset(this,0,sizeof(*this));
        }
    }  __attribute__((packed)) velodyne_data_packet_t;
    
    typedef struct velodyne_orientation {
        /* the values are only stored in the least significant 12 bits */
        uint16_t gyro; // 0.09766 deg/sec scale factor
        uint16_t temperature; // 0.1453 scale factor + 25°C offset
        uint16_t accel_x; // 0.001221 G scale factor
        uint16_t accel_y; // 0.001221 G scale factor
    }  __attribute__((packed)) velodyne_orientation_t;
    
    typedef struct velodyne_positioning_packet {
        uint8_t unused_1[14];
        velodyne_orientation_t orientations[VELODYNE_ORIENTATION_READINGS];
        uint8_t unused_2[160];
        uint32_t gps_timestamp; // in microseconds from the top of the hour
        uint8_t unused_3[4];
        char nmea_sentence[72];
        uint8_t unused_4[234];
        
        velodyne_positioning_packet()
        {
            //clear everything 
            memset(this,0,sizeof(*this));
        }
    }  __attribute__((packed)) velodyne_positioning_packet_t;

};

#endif
