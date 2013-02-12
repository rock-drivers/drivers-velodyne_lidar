#ifndef velodyne_lidar_MultilevelLaserVisualization_H
#define velodyne_lidar_MultilevelLaserVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <velodyne/MultilevelLaserScan.h>

namespace vizkit
{
    class MultilevelLaserVisualization
        : public vizkit::Vizkit3DPlugin<velodyne::MultilevelLaserScan>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        MultilevelLaserVisualization();
        ~MultilevelLaserVisualization();

    Q_INVOKABLE void updateData(velodyne::MultilevelLaserScan const &sample)
    {vizkit::Vizkit3DPlugin<velodyne::MultilevelLaserScan>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(velodyne::MultilevelLaserScan const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
