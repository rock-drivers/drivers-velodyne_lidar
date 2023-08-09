# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(velodyne_lidar 0.1)
rock_feature(NOCURDIR)
rock_standard_layout()
