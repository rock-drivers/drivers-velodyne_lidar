#ifndef _VELODYNE_POSITIONING_DRIVER_HPP_
#define _VELODYNE_POSITIONING_DRIVER_HPP_

#include <iodrivers_base/Driver.hpp>
#include <velodyne_lidar/velodyneProtocolTypes.hpp>
#include <velodyne_lidar/gps_rmc_type.h>
#include <string.h>

namespace velodyne_lidar
{

class VelodynePositioningDriver : public iodrivers_base::Driver 
{
public:
    VelodynePositioningDriver();

    /**
     * prints the packet
     */
    void print_packet(velodyne_positioning_packet_t &);

    /**
    * Helper method from Driver to extract the desired message.
    */
    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
    
    /**
     * Converts the internal type velodyne_orientation_t to a two dimensional accelerometer, 
     * a one dimensional gyrometer and one temperature measurement
     */
    void convertMotionValues(const velodyne_orientation_t& velodyne_orientation_data, double& gyro, double& temp, double& accel_x, double& accel_y) const;
    
    /**
     * Converts the internal type velodyne_orientation_t array to a calibrated gyroscope, accelerometer and temperature measurement vector.
     * Gyroscope measurements are in rad/s, accelerometer in m/s^2, temperature in degrees Celsius.
     */
    void convertIMUReadingsCalibrated(const velodyne_orientation_t rawdata[VELODYNE_ORIENTATION_READINGS], double gyro[3], double accel[3], double temp[3]) const;

    /**
     * Converts a NMEA GPRMC sentence to a appropriate data struct
     */
    void convertNMEASentence(const std::string& nmea_message, GPS_RMC& rmc_data);
    
protected:
    /**
     * Checks the crc value in a NMEA sentence
     */
    bool checkNMEA_CRC(const std::string& mnea_message);
    
    /**
     * Creates a base::Time from two given date and time strings
     */
    base::Time getGPSDateTime(const std::string& date, const std::string& time);
    
};

};

#endif
