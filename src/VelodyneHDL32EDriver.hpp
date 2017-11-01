#ifndef _VELODYNE_HDL32E_DRIVER_HPP_
#define _VELODYNE_HDL32E_DRIVER_HPP_

#include "velodyneDataDriver.hpp"

namespace velodyne_lidar
{

class VelodyneHDL32EDriver : public VelodyneDataDriver
{
public:
    VelodyneHDL32EDriver();
    virtual ~VelodyneHDL32EDriver();

   /**
    * Converts a full scan to a DepthMap sample.
    * Also clears all collected packets of the corresponding laser head.
    * @returns true on success, false if nothing to convert.
    */
    virtual bool convertScanToSample(base::samples::DepthMap& sample, bool copy_remission = true);

   /**
    * Return frequency of the arriving data samples
    */
    virtual double getBroadcastFrequency();
};

}

#endif