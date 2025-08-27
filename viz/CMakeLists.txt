rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

rock_qt_vizkit_plugin(
    TARGETPREFIX velodyne_lidar-viz
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES MultilevelLaserVisualization.cpp
    DEPS velodyne_lidar
    DEPS_PKGCONFIG_QT base-viz
    MOC MultilevelLaserVisualization.hpp
    HEADERS MultilevelLaserVisualization.hpp)

