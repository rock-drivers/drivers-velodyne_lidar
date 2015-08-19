rock_library(velodyne_lidar
    SOURCES velodyneDataDriver.cpp velodynePositioningDriver.cpp pointcloudConvertHelper.cpp
    HEADERS velodyneDataDriver.hpp velodynePositioningDriver.hpp pointcloudConvertHelper.hpp velodyneConstants.hpp velodyneProtocolTypes.hpp MultilevelLaserScan.h gps_rmc_type.h
    DEPS_PKGCONFIG base-lib iodrivers_base)

rock_executable(velodyne_lidar_bin test_velodyneDriver.cpp
    DEPS velodyne_lidar)
    
rock_executable(velodyne_lidar_position_bin test_velodynePositioningDriver.cpp
    DEPS velodyne_lidar)

