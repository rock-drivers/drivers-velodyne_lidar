rock_library(velodyne_lidar
    SOURCES velodyneDataDriver.cpp
    HEADERS velodyneDataDriver.hpp velodyneConstants.hpp velodyneProtocolTypes.hpp MultilevelLaserScan.h
    DEPS_PKGCONFIG base-lib iodrivers_base)

rock_executable(velodyne_lidar_bin test_velodyneDriver.cpp
    DEPS velodyne_lidar)

