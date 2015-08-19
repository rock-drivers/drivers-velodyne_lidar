rock_vizkit_plugin(velodyne_lidar-viz
    MultilevelLaserVisualization.cpp
    DEPS velodyne_lidar
    DEPS_PKGCONFIG base-viz
    MOC MultilevelLaserVisualization.hpp
    HEADERS MultilevelLaserVisualization.hpp)

