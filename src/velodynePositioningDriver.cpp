#include "velodynePositioningDriver.hpp"

#include <iostream>
#include <assert.h>
#include <sys/time.h>
#include <time.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>

using namespace velodyne_lidar;

VelodynePositioningDriver::VelodynePositioningDriver() : Driver(VELODYNE_POSITIONING_MSG_BUFFER_SIZE)
{
    assert(sizeof(velodyne_positioning_packet_t) == VELODYNE_POSITIONING_MSG_BUFFER_SIZE);
}

int VelodynePositioningDriver::extractPacket(const uint8_t* buffer, size_t buffer_size) const
{
    if(buffer_size == VELODYNE_POSITIONING_MSG_BUFFER_SIZE)
        return buffer_size;
    
    return -buffer_size;
}

void VelodynePositioningDriver::convertMotionValues(const velodyne_orientation_t& velodyne_orientation_data, double& gyro, double& temp, double& accel_x, double& accel_y) const
{
    assert(((velodyne_orientation_data.gyro & 0xF000)>>12) == 0);
    assert(((velodyne_orientation_data.temperature & 0xF000)>>12) == 1);
    assert(((velodyne_orientation_data.accel_x & 0xF000)>>12) == 2);
    assert(((velodyne_orientation_data.accel_y & 0xF000)>>12) == 3);
    
    gyro = ( (double) (( (int16_t)(velodyne_orientation_data.gyro<<4) )/16) ) * 0.09766;
    temp = ( (double) (( (int16_t)(velodyne_orientation_data.temperature<<4) )/16) ) * 0.1453 + 25.0;
    accel_x = ( (double) (( (int16_t)(velodyne_orientation_data.accel_x<<4) )/16) ) * 0.001221;
    accel_y = ( (double) (( (int16_t)(velodyne_orientation_data.accel_y<<4) )/16) ) * 0.001221;
}

inline double scale_sign_extend_int12(uint16_t raw, double scale = 1.0)
{
    return int16_t(raw<<4) * (scale/16);
}

void VelodynePositioningDriver::convertIMUReadingsCalibrated(const velodyne_orientation_t rawdata[VELODYNE_ORIENTATION_READINGS], double gyro[3], double accel[3], double temp[3]) const
{
    for(int i=0; i<3; ++i)
    {
        gyro[i] = scale_sign_extend_int12(rawdata[i].gyro, 0.09766 * M_PI/180);
        temp[i] = scale_sign_extend_int12(rawdata[i].temperature, 0.1453) + 25.0;
    }
    // average measurements of colinear accelerometers:
    const double accel_scale = 0.001221*9.81/2;
    accel[0] = scale_sign_extend_int12(rawdata[1].accel_y, -accel_scale) +  scale_sign_extend_int12(rawdata[2].accel_y, -accel_scale);
    accel[1] = scale_sign_extend_int12(rawdata[0].accel_y, -accel_scale) +  scale_sign_extend_int12(rawdata[2].accel_x,  accel_scale);
    accel[2] = scale_sign_extend_int12(rawdata[0].accel_x,  accel_scale) +  scale_sign_extend_int12(rawdata[1].accel_x,  accel_scale);
}



void VelodynePositioningDriver::convertNMEASentence(const std::string& nmea_message, GPS_RMC& rmc_data)
{
    if(nmea_message.size() == 0)
        throw std::runtime_error("no NMEA sentence received, message is empty");
    else if(nmea_message.find("$GPRMC,") != 0)
        throw std::runtime_error("wrong message given to convertNMEASentence: " + nmea_message);

    std::vector<std::string> fields;
    boost::split( fields, nmea_message, boost::is_any_of(",*") );
    
    // TODO: the CRC values of the garmin gps are wrong or calculated in a different way, so the crc check is deactivated for the moment.
    // check crc value
    if(fields.size() == 14)// && checkNMEA_CRC(mnea_message))
    {
        rmc_data.utc_time = getGPSDateTime(fields[9], fields[1]);
        char status = *fields[2].begin();
        switch (status)
        {
            case 'A':
                rmc_data.status = ValidPosition;
                break;
            case 'V':
                rmc_data.status = ReceiverWarning;
                break;
            default:
                rmc_data.status = UnknownStatus;
        }
        rmc_data.latitude = (fields[3].size() == 0 ? 0.0 : boost::lexical_cast<double>(fields[3]));
        char latitude_hemisphere = *fields[4].begin();
        switch (latitude_hemisphere)
        {
            case 'N':
                rmc_data.latitude_hemisphere = North;
                break;
            case 'S':
                rmc_data.latitude_hemisphere = South;
                break;
            default:
                rmc_data.latitude_hemisphere = UnknownHemisphere;
        }
        rmc_data.longitude = (fields[5].size() == 0 ? 0.0 : boost::lexical_cast<double>(fields[5]));
        char longitude_hemisphere = *fields[6].begin();
        switch (longitude_hemisphere)
        {
            case 'E':
                rmc_data.longitude_hemisphere = East;
                break;
            case 'W':
                rmc_data.longitude_hemisphere = West;
                break;
            default:
                rmc_data.longitude_hemisphere = UnknownHemisphere;
        }
        rmc_data.speed = (fields[7].size() == 0 ? 0.0 : boost::lexical_cast<double>(fields[7]));
        rmc_data.angle = (fields[8].size() == 0 ? 0.0 : boost::lexical_cast<double>(fields[8]));
        rmc_data.magnetic_variation = (fields[10].size() == 0 ? 0.0 : boost::lexical_cast<double>(fields[10]));
        char mv_hemisphere = *fields[11].begin();
        switch (mv_hemisphere)
        {
            case 'E':
                rmc_data.mv_hemisphere = East;
                break;
            case 'W':
                rmc_data.mv_hemisphere = West;
                break;
            default:
                rmc_data.mv_hemisphere = UnknownHemisphere;
        }
        char signal_mode = *fields[12].begin();
        switch (signal_mode)
        {
            case 'A':
                rmc_data.signal_mode = AutonomousMode;
                break;
            case 'D':
                rmc_data.signal_mode = DifferentialMode;
                break;
            case 'E':
                rmc_data.signal_mode = EstimatedMode;
                break;
            case 'N':
                rmc_data.signal_mode = InvalidMode;
                break;
            case 'S':
                rmc_data.signal_mode = SimulatedMode;
                break;
            default:
                rmc_data.signal_mode = UnknownMode;
        }
    }
    else
    {
        throw std::runtime_error("CRC error in mnea sentence.");
    }
}

bool VelodynePositioningDriver::checkNMEA_CRC(const std::string& mnea_message)
{
    std::vector<std::string> fields;
    boost::split( fields, mnea_message, boost::is_any_of("$*") );
    
    if(fields.size() == 3)
    {
        char* endptr = 0;
        long unsigned int crc_value = strtoul(fields[2].c_str(), &endptr, 16);
        const char* message = fields[1].c_str();
        uint8_t crc_checksum = 0;
        
        for(unsigned i = 0; i < fields[1].size(); i++)
        {
            crc_checksum ^= message[i];
        }
        
        if(crc_value == crc_checksum)
        {
            return true;
        }
    }
    
    return false;
}

base::Time VelodynePositioningDriver::getGPSDateTime(const std::string& date, const std::string& time)
{
    float gps_time   = atof(time.c_str());
    int integer_part = gps_time;
    int microsecs = (gps_time - integer_part) * 1000000;
    int year = atoi(std::string(date, 4, 2).c_str());
    if(year >= 70)
        year += 1900;
    else
        year += 2000;
    
    return base::Time::fromTimeValues(year,
                                      atoi(std::string(date, 2, 2).c_str()),
                                      atoi(std::string(date, 0, 2).c_str()),
                                      integer_part / 10000,
                                      (integer_part / 100) % 100,
                                      (integer_part % 100),
                                      microsecs / 1000,
                                      microsecs % 1000);
}

void VelodynePositioningDriver::print_packet(velodyne_positioning_packet_t& packet)
{
    std::cout << "GPS-Timestamp: " << packet.gps_timestamp << std::endl << std::endl;
    
    for(unsigned i = 0; i < VELODYNE_ORIENTATION_READINGS; i++)
    {
        double gyro, temp, accel_x, accel_y;
        convertMotionValues(packet.orientations[i], gyro, temp, accel_x, accel_y);
        std::cout << "Messurment " << i+1 << ":" << std::endl;
        std::cout << "Gyro: " << gyro << ", Temperature: " << temp << ", Accel X: " << accel_x << ", Accel Y: " << accel_y << std::endl;
    }
    std::cout << std::endl;
    
    std::cout << "Raw NMEA Sentence: " << packet.nmea_sentence << std::endl;
    
    GPS_RMC rmc_data;
    convertNMEASentence(std::string(packet.nmea_sentence), rmc_data);
    std::cout << "UTC-Time: " << rmc_data.utc_time << std::endl;
    std::cout << "Status: " << (rmc_data.status == 'A' ? "Valid Position" : "Warrning, invalid Position") << std::endl;
    std::cout << "Latitude: " << rmc_data.latitude << ", Hemisphere: " << rmc_data.latitude_hemisphere << std::endl;
    std::cout << "Longitude: " << rmc_data.longitude << ", Hemisphere: " << rmc_data.longitude_hemisphere << std::endl;
    std::cout << "Speed over ground: " << rmc_data.speed << ", track angle over ground: " << rmc_data.angle << std::endl;
    std::cout << "Magnetic variation: " << rmc_data.magnetic_variation << ", magnetic variation direction: " << rmc_data.mv_hemisphere << std::endl;
    std::cout << "Signal mode: " << rmc_data.signal_mode << std::endl;
    
    std::cout << std::endl << std::endl;
}
