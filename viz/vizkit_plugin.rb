Vizkit::UiLoader.register_3d_plugin('MultilevelLaserVisualization', 'velodyne_lidar', 'MultilevelLaserVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MultilevelLaserVisualization', "/velodyne_lidar/MultilevelLaserScan", :updateLaserScan )
Vizkit::UiLoader.register_3d_plugin_for('MultilevelLaserVisualization', "/base/samples/RigidBodyState", :updatePose )