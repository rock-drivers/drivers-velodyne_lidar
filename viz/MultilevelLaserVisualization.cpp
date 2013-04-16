#include <iostream>
#include <vizkit/Vizkit3DHelper.hpp>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include "MultilevelLaserVisualization.hpp"
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <time.h>
#include <vizkit/ColorConversionHelper.hpp>

using namespace vizkit;

MultilevelLaserVisualization::MultilevelLaserVisualization() : 
    skip_n_horizontal_scans(0), colorize_altitude(false), colorize_magnitude(false), colorize_interval(1.0)
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
    velodyne_lidar::ConvertHelper::convertScanToPointCloud(scan, points,Eigen::Affine3d::Identity(), true, skip_n_horizontal_scans);
    
    //set color binding
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    if(colorize_magnitude || colorize_altitude)
    {
        for(std::vector<Eigen::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++)
        {
            double hue = 0.0;
            if(colorize_altitude)
                hue = (it->z() - std::floor(it->z() / colorize_interval) * colorize_interval) / colorize_interval;
            else
                hue = (it->norm() - std::floor(it->norm() / colorize_interval) * colorize_interval) / colorize_interval;
            osg::Vec4 color( 1.0, 1.0, 1.0, 1.0 );
            hslToRgb(hue, 1.0, 0.5, color.r(), color.g(), color.b());
            colors->push_back(color);
        }
        scanGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    }
    else
    {
        colors->push_back(osg::Vec4(0,0,0.3,0.5));
        colors->push_back(osg::Vec4(1,0,0,1));
        scanGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    }
    scanGeom->setColorArray(colors);
    
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

int MultilevelLaserVisualization::getSkipHorizontalScans() const
{
    return skip_n_horizontal_scans;
}

void MultilevelLaserVisualization::setSkipHorizontalScans(int count)
{
    if(count >= 0)
    {
        skip_n_horizontal_scans = count;
        emit propertyChanged("SkipHorizontalScans");
    }
}

double MultilevelLaserVisualization::getColorizeInterval() const
{
    return colorize_interval;
}

void MultilevelLaserVisualization::setColorizeInterval(double value)
{
    if(value != 0.0)
    {
        colorize_interval = value;
        emit propertyChanged("ColorizeInterval");
    }
}

bool MultilevelLaserVisualization::isColorizeAltitudeEnabled() const
{
    return colorize_altitude;
}

void MultilevelLaserVisualization::setColorizeAltitude(bool value)
{
    colorize_altitude = value;
    emit propertyChanged("ColorizeAltitude");
}

bool MultilevelLaserVisualization::isColorizeMagnitudeEnabled() const
{
    return colorize_magnitude;
}

void MultilevelLaserVisualization::setColorizeMagnitude(bool value)
{
    colorize_magnitude = value;
    emit propertyChanged("ColorizeMagnitude");
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(MultilevelLaserVisualization)

