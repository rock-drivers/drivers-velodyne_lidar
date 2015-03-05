#ifndef _VELODYNE_DATA_DRIVER_HPP_
#define _VELODYNE_DATA_DRIVER_HPP_

#include <iodrivers_base/Driver.hpp>
#include <velodyne_lidar/velodyneProtocolTypes.hpp>
#include <base/samples/DepthMap.hpp>

namespace velodyne_lidar
{

class VelodyneDataDriver : public iodrivers_base::Driver 
{
public:    
    VelodyneDataDriver();
			
	/**
	 * Collects a column of all distance (and optional remission) values from velodyne_fire_t into the matrix (matrices)
	 */
	void collectColumn(const velodyne_fire_t& velodyne_fire, Eigen::Matrix<base::samples::DepthMap::scalar, Eigen::Dynamic, Eigen::Dynamic> &distances, Eigen::Matrix<base::samples::DepthMap::scalar, Eigen::Dynamic, Eigen::Dynamic> &remissions, unsigned int col, bool useRemissions);
	
    /**
     * prints the packet
     */
    void print_packet(velodyne_data_packet_t &);

    /**
    * Helper method from Driver to extract the desired message.
    */
    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

};

};

#endif
