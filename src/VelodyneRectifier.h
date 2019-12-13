/*
 * VelodyneRectifier.h
 *
 *  Created on: Nov 29, 2019
 *      Author: marco, edit: jasmin
 */

#ifndef SRC_VELODYNERECTIFIER_H_
#define SRC_VELODYNERECTIFIER_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <tf_conversions/tf_eigen.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <geometry_msgs/PointStamped.h>

namespace fd_scanner
{

static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;        // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;    // [µs]

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
	uint16_t header;    ///< UPPER_BANK or LOWER_BANK
	uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
	uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
	uint16_t uint;
	uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
	raw_block_t blocks[BLOCKS_PER_PACKET];
	uint16_t revolution;
	uint8_t status[PACKET_STATUS_SIZE];
}
raw_packet_t;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * Class to rectify Velodyne point cloud during scanner movement. Interpolates the positions between the start
 * and end of recording a complete sweep to adjust the single point's positions.
 */
class VelodyneRectifier {
private:
	ros::NodeHandle nh_;
	ros::Subscriber velodyne_packet_sub_;
	ros::Publisher rectified_velodyne_pub_;
	tf::TransformListener tf_listener_;

	float vertical_correction_[16];
	float cos_vertical_correction_[16];
	float sin_vertical_correction_[16];
	float sin_rot_table_[ROTATION_MAX_UNITS];
	float cos_rot_table_[ROTATION_MAX_UNITS];
	// timing offset lookup table
	std::vector< std::vector<float> > timing_offsets;

public:
	VelodyneRectifier();
	virtual ~VelodyneRectifier();
	void init();
	void velodynePacketCallback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
	void unpack(const velodyne_msgs::VelodynePacket &pkt, PointCloud::Ptr msg);
};

}

#endif /* SRC_VELODYNERECTIFIER_H_ */
