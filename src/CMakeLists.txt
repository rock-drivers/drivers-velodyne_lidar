rock_library(velodyne_lidar
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(velodyne_lidar_bin Main.cpp
    DEPS velodyne_lidar)

