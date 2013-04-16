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
    
    Q_PROPERTY(int SkipHorizontalScans READ getSkipHorizontalScans WRITE setSkipHorizontalScans)
    Q_PROPERTY(bool ColorizeAltitude READ isColorizeAltitudeEnabled WRITE setColorizeAltitude)
    Q_PROPERTY(bool ColorizeMagnitude READ isColorizeMagnitudeEnabled WRITE setColorizeMagnitude)
    Q_PROPERTY(double ColorizeInterval READ getColorizeInterval WRITE setColorizeInterval)
    
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
        
    public slots:
        int getSkipHorizontalScans() const;
        void setSkipHorizontalScans(int count);
        void setColorizeAltitude(bool value);
        bool isColorizeAltitudeEnabled()const;
        void setColorizeMagnitude(bool value);
        bool isColorizeMagnitudeEnabled()const;
        void setColorizeInterval(double value);
        double getColorizeInterval()const;

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
        unsigned int skip_n_horizontal_scans;
        bool colorize_altitude;
        bool colorize_magnitude;
        double colorize_interval;
    };
}
#endif
