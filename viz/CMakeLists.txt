rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

if(ROCK_QT_VERSION_4)
    rock_vizkit_plugin(velodyne_lidar-viz
        MultilevelLaserVisualization.cpp
        DEPS velodyne_lidar
        DEPS_PKGCONFIG base-viz
        MOC MultilevelLaserVisualization.hpp
        HEADERS MultilevelLaserVisualization.hpp)
endif()

if(ROCK_QT_VERSION_5)
    rock_vizkit_plugin_qt5(velodyne_lidar-viz-qt5
        MultilevelLaserVisualization.cpp
        DEPS velodyne_lidar
        DEPS_PKGCONFIG base-viz-qt5
        MOC5 MultilevelLaserVisualization.hpp
        HEADERS MultilevelLaserVisualization.hpp)
endif()
