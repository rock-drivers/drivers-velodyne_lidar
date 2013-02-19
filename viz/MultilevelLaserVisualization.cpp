#include <iostream>
#include <vizkit/Vizkit3DHelper.hpp>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include "MultilevelLaserVisualization.hpp"
#include <time.h>

using namespace vizkit;

MultilevelLaserVisualization::MultilevelLaserVisualization()
{
    scanOrientation = Eigen::Quaterniond::Identity();
    scanPosition.setZero();
}

MultilevelLaserVisualization::~MultilevelLaserVisualization()
{
}

osg::ref_ptr<osg::Node> MultilevelLaserVisualization::createMainNode()
{
    transformNode = new osg::PositionAttitudeTransform();
    scanNode = new osg::Geode();
    transformNode->addChild(scanNode);
    
    scanGeom = new osg::Geometry();
    
    //set color binding
    osg::Vec4Array *colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(0,0,0.3,0.5));
    colors->push_back(osg::Vec4(1,0,0,1));
    scanGeom->setColorArray(colors);
    scanGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    //setup normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    scanGeom->setNormalArray(normals);
    scanGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    //set size
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(5.0);
    point->setDistanceAttenuation( osg::Vec3(1.0, 1.0, 1.0 ) );
    point->setMinSize( 3.0 );
    point->setMaxSize( 5.0 );
    scanGeom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );

    //turn on transparacny
    scanNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    scanNode->addDrawable(scanGeom);

    return transformNode;
}

void MultilevelLaserVisualization::updateMainNode ( osg::Node* node )
{
    transformNode->setPosition(eigenVectorToOsgVec3(scanPosition));
    transformNode->setAttitude(eigenQuatToOsgQuat(scanOrientation));
    
    osg::Vec3Array *scanVertices = new osg::Vec3Array();

    std::vector<Eigen::Vector3d> points;
    scan.convertScanToPointCloud(points);
    
    scanVertices->reserve(points.size());

    for(std::vector<Eigen::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++)
        scanVertices->push_back(eigenVectorToOsgVec3(*it));
    scanGeom->setVertexArray(scanVertices);

    while(!scanGeom->getPrimitiveSetList().empty())
        scanGeom->removePrimitiveSet(0);
   
    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scanVertices->size()));
}

void MultilevelLaserVisualization::updateDataIntern(velodyne_lidar::MultilevelLaserScan const& sample)
{
    scan = sample;
}

void MultilevelLaserVisualization::updateDataIntern(const base::samples::RigidBodyState& sample)
{
    scanOrientation = sample.orientation;
    scanPosition = sample.position;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(MultilevelLaserVisualization)

