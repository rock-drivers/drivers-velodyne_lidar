#include <iostream>
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include "MultilevelLaserVisualization.hpp"
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <time.h>
#include <vizkit3d/ColorConversionHelper.hpp>

using namespace vizkit3d;

MultilevelLaserVisualization::MultilevelLaserVisualization() : 
    skip_n_horizontal_scans(0), colorize_altitude(false), colorize_magnitude(false), colorize_interval(1.0), show_remission(false), show_slope(false)
{
    scanOrientation = Eigen::Quaterniond::Identity();
    scanPosition.setZero();
    maximum_angle_to_neighbor = base::Angle::fromDeg(160);
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
    
    //setup slope geometry
    slopeGeom = new osg::Geometry();
    slopeGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    scanNode->addDrawable(slopeGeom);

    return transformNode;
}

void MultilevelLaserVisualization::updateMainNode ( osg::Node* node )
{
    transformNode->setPosition(eigenVectorToOsgVec3(scanPosition));
    transformNode->setAttitude(eigenQuatToOsgQuat(scanOrientation));
    
    osg::Vec3Array *scanVertices = new osg::Vec3Array();

    std::vector<Eigen::Vector3d> points;
    std::vector<float> remission_values;
    velodyne_lidar::MultilevelLaserScan filtered_scan;
    velodyne_lidar::ConvertHelper::filterOutliers(scan, filtered_scan, maximum_angle_to_neighbor.getRad());
    scan = filtered_scan;
    velodyne_lidar::ConvertHelper::convertScanToPointCloud(scan, points, Eigen::Affine3d::Identity(), true, skip_n_horizontal_scans, &remission_values);
    
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
	scanGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    }
    else if(show_remission)
    {
        for(std::vector<float>::const_iterator it = remission_values.begin(); it != remission_values.end(); it++)
        {
            colors->push_back(osg::Vec4(0,0,*it,0.5));
        }
	scanGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    }
    else
    {
        colors->push_back(osg::Vec4(0,0,0.3,0.5));
	scanGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
    }
    
    scanVertices->reserve(points.size());

    for(std::vector<Eigen::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++)
        scanVertices->push_back(eigenVectorToOsgVec3(*it));
    scanGeom->setVertexArray(scanVertices);

    while(!scanGeom->getPrimitiveSetList().empty())
        scanGeom->removePrimitiveSet(0);
   
    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scanVertices->size()));
    
    while(!slopeGeom->getPrimitiveSetList().empty())
        slopeGeom->removePrimitiveSet(0);
    
    //draw slope geometry
    if(show_slope)
    {
        osg::ref_ptr<osg::Vec3Array> slope_vertices = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4Array> slope_colors = new osg::Vec4Array();
        
        unsigned int h_skip_count = 0;
        int pointcloud_index = -1;
        for(std::vector<velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan>::const_iterator v_scan = scan.horizontal_scans.begin(); v_scan < scan.horizontal_scans.end(); v_scan++) 
        {
            // check if horizontal scan should be skipped
            if(h_skip_count < skip_n_horizontal_scans)
            {
                h_skip_count++;
                continue;
            }
            h_skip_count = 0;
            
            for(unsigned int i = 0; i < v_scan->vertical_scans.size(); i++)
            {
                if(scan.isRangeValid(v_scan->vertical_scans[i].range))
                    pointcloud_index++;
                
                if((i+1) >= v_scan->vertical_scans.size())
                    break;
                
                if(scan.isRangeValid(v_scan->vertical_scans[i].range) && scan.isRangeValid(v_scan->vertical_scans[i+1].range) &&
                    (std::min(v_scan->vertical_scans[i].range, v_scan->vertical_scans[i+1].range) * 1.3) >= std::max(v_scan->vertical_scans[i].range, v_scan->vertical_scans[i+1].range))
                {
                    slope_vertices->push_back(scanVertices->at(pointcloud_index));
                    slope_vertices->push_back(scanVertices->at(pointcloud_index+1));
                    
                    Eigen::Vector3d v_diff = points[pointcloud_index+1] - points[pointcloud_index];
                    Eigen::Vector3d v_ground = Eigen::Vector3d(v_diff.x(), v_diff.y(), 0.0);
                    double v_angle = acos((v_diff.dot(v_ground)) / (v_diff.norm() * v_ground.norm()));
                    
                    osg::Vec4 color( 1.0, 1.0, 1.0, 1.0 );
                    hslToRgb(v_angle/M_PI, 1.0, 0.5, color.r(), color.g(), color.b());
                    slope_colors->push_back(color);
                    slope_colors->push_back(color);
                }
            }
        }
        
        slopeGeom->setColorArray(slope_colors);
        slopeGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        slopeGeom->setVertexArray(slope_vertices);
        slopeGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,slope_vertices->size()));
    }
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

bool MultilevelLaserVisualization::isShowRemissionEnabled() const
{
    return show_remission;
}

void MultilevelLaserVisualization::setShowRemission(bool value)
{
    show_remission = value;
    emit propertyChanged("ShowRemission");
}

bool MultilevelLaserVisualization::isShowSlopeEnabled() const
{
    return show_slope;
}

void MultilevelLaserVisualization::setShowSlope(bool value)
{
    show_slope = value;
    emit propertyChanged("ShowSlope");
}

double MultilevelLaserVisualization::getMaximumAngleToNeighbor() const
{
    return maximum_angle_to_neighbor.getDeg();
}

void MultilevelLaserVisualization::setMaximumAngleToNeighbor(double value)
{
    maximum_angle_to_neighbor = base::Angle::fromDeg(value);
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(MultilevelLaserVisualization)

