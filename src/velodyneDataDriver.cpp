// Software License Agreement (BSD License)
//
// Copyright (c) 2008, Tully Foote
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "velodyneDataDriver.hpp"

using namespace velodyne_lidar;

VelodyneDataDriver::VelodyneDataDriver() : Driver(VELODYNE_DATA_MSG_BUFFER_SIZE) 
{
  // Confirm that the UDP message buffer matches the structure size
  assert(sizeof(velodyne_data_packet_t) == VELODYNE_DATA_MSG_BUFFER_SIZE);
}

void VelodyneDataDriver::convertToVerticalMultilevelScan(const velodyne_fire& velodyne_fire, MultilevelLaserScan::VerticalMultilevelScan& vertical_scan)
{
    if(vertical_scan.vertical_scans.size() != VELODYNE_NUM_LASERS)
        vertical_scan.vertical_scans.resize(VELODYNE_NUM_LASERS);
    
    for(unsigned i = 0; i < VELODYNE_NUM_LASERS; i++)
    {
        convertToSingleScan(velodyne_fire.lasers[VELODYNE_FIRING_ORDER[i]], vertical_scan.vertical_scans[i]);
    }
    
    vertical_scan.horizontal_angle = base::Angle::fromDeg(((double)velodyne_fire.rotational_pos) * 0.01);
    vertical_scan.vertical_start_angle = base::Angle::fromDeg(VELODYNE_VERTICAL_START_ANGLE);
    vertical_scan.vertical_angular_resolution = base::Angle::deg2Rad(VELODYNE_VERTICAL_RESOLUTION);
}

void VelodyneDataDriver::convertToSingleScan(const vel_laser_t& velodyne_laser, MultilevelLaserScan::SingleScan& single_scan)
{
    single_scan.range = velodyne_laser.distance == 0 ? MultilevelLaserScan::TOO_FAR  : velodyne_laser.distance * 2; // 2mm to 1mm increment
    single_scan.remission = ((float)velodyne_laser.intensity) / 255.0f;
}

int VelodyneDataDriver::extractPacket(const uint8_t* buffer, size_t buffer_size) const
{
    if(buffer_size == VELODYNE_DATA_MSG_BUFFER_SIZE)
        return buffer_size;
    
    return -buffer_size;
}

void VelodyneDataDriver::print_packet(velodyne_data_packet_t &packet) 
{
    for (int in = 0; in < 12 ; in++)
    {
        printf("Block %d, %d, %d:\n",in, packet.shots[in].lower_upper, packet.shots[in].rotational_pos);
        printf("Distance: ");
        for (int im = 0; im < 32 ; im ++)
        {
            printf("%d\t", packet.shots[in].lasers[im].distance);
        }
        printf("\nIntensity: ");
        for (int im = 0; im < 32 ; im ++)
        {
            printf("%d\t" , packet.shots[in].lasers[im].intensity);
        }
        printf("\n");
    }
    printf("Status: \n");
    
    std::cout<< "Type:"<< (double)packet.status_type  << " Value: " <<(double)packet.status_value  <<" GPS Timestamp: " << packet.gps_timestamp << std::endl;
    base::Time gps_time = base::Time::fromMicroseconds(packet.gps_timestamp);
    std::cout<< "GPS Time:" << gps_time.toString() << std::endl;
    printf("\n\n\n\n");
}
