#include <iostream>
#include "MultilevelLaserVisualization.hpp"

using namespace vizkit;

struct MultilevelLaserVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    velodyne::MultilevelLaserScan data;
};


MultilevelLaserVisualization::MultilevelLaserVisualization()
    : p(new Data)
{
}

MultilevelLaserVisualization::~MultilevelLaserVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> MultilevelLaserVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void MultilevelLaserVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void MultilevelLaserVisualization::updateDataIntern(velodyne::MultilevelLaserScan const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(MultilevelLaserVisualization)

