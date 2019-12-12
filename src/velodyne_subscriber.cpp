/*
 * velodyne_subscriber.cpp
 *
 *  Created on: Nov 15, 2019
 *      Author: jasmin
 */

#include "ros/ros.h"
#include "velodyne_msgs/VelodyneScan.h"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <iomanip>
//#include <velodyne_pointcloud/rawdata.h>
//#include <velodyne_pointcloud/convert.h>

std::vector<velodyne_msgs::VelodyneScan> vecVelodyneScan;
std::vector<velodyne_msgs::VelodynePacket> vecVelodynePackets;

//boost::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);
/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;

typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
}
raw_packet_t;

void writeDataToFile()
{
  //open file
  std::ofstream data_file("/home/jasmin/workspace/ros/src/velodyne_sub/my_data_file.txt");
  if(data_file.is_open())
  {
    for(int i = 0; i < vecVelodyneScan.size(); i++)
    {
//      data_file << vecVelodyneScan[i] << std::endl;
    }
  }
  else
  {
    std::cout << "Cannot open file. " << std::endl;
    exit(1);
  }
  data_file.close();
}


void unpack_vlp16(const velodyne_msgs::VelodynePacket& pkt)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  const raw_packet_t* raw = (const raw_packet_t*) &pkt.data[0];
  float azimuth = 0.0;
  int raw_azimuth_diff = 0;
  float azimuth_diff = 0.0;
  float last_azimuth_diff = 0;
  float first_azimuth_angle = 0.0;
  float maxAzimuthDiff = 360.0;

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                               << block << " header value is "
                               << raw->blocks[block].header);
      return;                         // bad packet: skip the rest
    }

    //erster azimuth -- nach 360° aufhören
    first_azimuth_angle = (float)(raw->blocks[0].rotation);

    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw->blocks[block].rotation);

    while((azimuth - first_azimuth_angle) <= 360.0)
    {

    if (block < (BLOCKS_PER_PACKET-1))
    {
        raw_azimuth_diff = raw->blocks[block+1].rotation - raw->blocks[block].rotation;
        azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);
        // some packets contain an angle overflow where azimuth_diff < 0
        if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
        {
          ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block+1].rotation);
          // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
          if(last_azimuth_diff > 0){
            azimuth_diff = last_azimuth_diff;
          }
          // otherwise we are not able to use this data
          // TODO: we might just not use the second 16 firings
          else
          {
            continue;
          }
        }
      last_azimuth_diff = azimuth_diff;
    }
    else
    {
      azimuth_diff = last_azimuth_diff;
    }

//    for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
//      for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
//        velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];
//
//        /** Position Calculation */
//        union two_bytes tmp;
//        tmp.bytes[0] = raw->blocks[block].data[k];
//        tmp.bytes[1] = raw->blocks[block].data[k+1];
//
//        /** correct for the laser rotation as a function of timing during the firings **/
//        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
//        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
//
//
//
//          // convert polar coordinates to Euclidean XYZ
//          float distance = tmp.uint * calibration_.distance_resolution_m;
//          distance += corrections.dist_correction;
//
//          float cos_vert_angle = corrections.cos_vert_correction;
//          float sin_vert_angle = corrections.sin_vert_correction;
//          float cos_rot_correction = corrections.cos_rot_correction;
//          float sin_rot_correction = corrections.sin_rot_correction;
//
//          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
//          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
//          float cos_rot_angle =
//            cos_rot_table_[azimuth_corrected] * cos_rot_correction +
//            sin_rot_table_[azimuth_corrected] * sin_rot_correction;
//          float sin_rot_angle =
//            sin_rot_table_[azimuth_corrected] * cos_rot_correction -
//            cos_rot_table_[azimuth_corrected] * sin_rot_correction;
//
//          float horiz_offset = corrections.horiz_offset_correction;
//          float vert_offset = corrections.vert_offset_correction;
//
//          // Compute the distance in the xy plane (w/o accounting for rotation)
//          /**the new term of 'vert_offset * sin_vert_angle'
//           * was added to the expression due to the mathemathical
//           * model we used.
//           */
//          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
//
//          // Calculate temporal X, use absolute value.
//          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
//          // Calculate temporal Y, use absolute value
//          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
//          if (xx < 0) xx=-xx;
//          if (yy < 0) yy=-yy;
//
//          // Get 2points calibration values,Linear interpolation to get distance
//          // correction for X and Y, that means distance correction use
//          // different value at different distance
//          float distance_corr_x = 0;
//          float distance_corr_y = 0;
//          if (corrections.two_pt_correction_available) {
//            distance_corr_x =
//              (corrections.dist_correction - corrections.dist_correction_x)
//                * (xx - 2.4) / (25.04 - 2.4)
//              + corrections.dist_correction_x;
//            distance_corr_x -= corrections.dist_correction;
//            distance_corr_y =
//              (corrections.dist_correction - corrections.dist_correction_y)
//                * (yy - 1.93) / (25.04 - 1.93)
//              + corrections.dist_correction_y;
//            distance_corr_y -= corrections.dist_correction;
//          }
//
//          float distance_x = distance + distance_corr_x;
//          /**the new term of 'vert_offset * sin_vert_angle'
//           * was added to the expression due to the mathemathical
//           * model we used.
//           */
//          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
//          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
//
//          float distance_y = distance + distance_corr_y;
//          /**the new term of 'vert_offset * sin_vert_angle'
//           * was added to the expression due to the mathemathical
//           * model we used.
//           */
//          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
//          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
//
//          // Using distance_y is not symmetric, but the velodyne manual
//          // does this.
//          /**the new term of 'vert_offset * cos_vert_angle'
//           * was added to the expression due to the mathemathical
//           * model we used.
//           */
//          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
//
//          }
//        }
      std::cout << "azimuth = " << azimuth << "in block = " << block << std::endl;
    }//while <360°

  }//for
}//unpack

void velodyneCallback(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  for(int i = 0; i < scanMsg->packets.size(); i++)
  {
  unpack_vlp16(scanMsg->packets[i]);
  }
}



int main(int argc, char **argv)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  ros::init(argc, argv, "velodyne_sub");

  ros::NodeHandle nh;

  ros::Subscriber subVelo = nh.subscribe("velodyneScanTopic", 1000, velodyneCallback);

  ros::spin();

  return 0;
}


