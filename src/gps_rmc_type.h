#ifndef _GPS_RMC_TYPE_H_
#define _GPS_RMC_TYPE_H_

#include <base/Time.hpp>

namespace velodyne_lidar
{
    
enum GPSStatus
{
    ValidPosition,
    ReceiverWarning,
    UnknownStatus
};

enum SignalMode
{
    AutonomousMode,
    DifferentialMode,
    EstimatedMode,
    InvalidMode,
    SimulatedMode,
    UnknownMode
};

enum Hemisphere
{
    North,
    East,
    South,
    West,
    UnknownHemisphere
};
    
struct GPS_RMC
{
    base::Time utc_time; 
    GPSStatus status;                   // gps status
    double latitude;                    // latitude in NDEG - [degree][min].[sec/60]
    Hemisphere latitude_hemisphere;     // latitude hemisphere, north or south
    double longitude;                   // longitude in NDEG - [degree][min].[sec/60]
    Hemisphere longitude_hemisphere;    // longitude hemisphere, east or west
    double speed;                       // speed over ground in knots
    double angle;                       // track angle over ground in degrees
    double magnetic_variation;          // magnetic variation in degrees
    Hemisphere mv_hemisphere;           // hemisphere of magnetic variation, east or west 
    SignalMode signal_mode;             // signal integrity (autonomous, differential, estimated, not valid, simulated)
};

};

#endif
