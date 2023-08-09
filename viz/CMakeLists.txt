rock_find_qt5(Core)
rock_vizkit_plugin_qt5(velodyne_lidar-viz-qt5
    MultilevelLaserVisualization.cpp
    DEPS velodyne_lidar
    DEPS_PKGCONFIG base-viz-qt5
    MOC5 MultilevelLaserVisualization.hpp
    HEADERS MultilevelLaserVisualization.hpp)

