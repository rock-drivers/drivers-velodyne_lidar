#ifndef velodyne_lidar_MultilevelLaserVisualization_H
#define velodyne_lidar_MultilevelLaserVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/rigid_body_state.h>
#include <velodyne_lidar/MultilevelLaserScan.h>

namespace vizkit
{
    class MultilevelLaserVisualization
        : public vizkit::Vizkit3DPlugin<velodyne_lidar::MultilevelLaserScan>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        MultilevelLaserVisualization();
        ~MultilevelLaserVisualization();
        
        Q_INVOKABLE void updateData(const velodyne_lidar::MultilevelLaserScan& data)
        { Vizkit3DPlugin<velodyne_lidar::MultilevelLaserScan>::updateData(data); }
        Q_INVOKABLE void updateLaserScan(const velodyne_lidar::MultilevelLaserScan& data)
        { updateData(data); }
        Q_INVOKABLE void updateData(const base::samples::RigidBodyState& data)
        { Vizkit3DPlugin<velodyne_lidar::MultilevelLaserScan>::updateData(data); }
        Q_INVOKABLE void updatePose(const base::samples::RigidBodyState& data)
        { updateData(data); }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(velodyne_lidar::MultilevelLaserScan const& sample);
        virtual void updateDataIntern(const base::samples::RigidBodyState& sample);
        
    private:
        velodyne_lidar::MultilevelLaserScan scan;
        Eigen::Vector3d scanPosition;
        Eigen::Quaterniond scanOrientation;
        osg::ref_ptr< osg::PositionAttitudeTransform > transformNode;
        osg::ref_ptr<osg::Geode> scanNode;
        osg::ref_ptr<osg::Geometry> scanGeom;
    };
}
#endif
