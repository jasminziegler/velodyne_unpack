/*
 * VelodyneRectifier.cpp
 *
 *  Created on: Nov 29, 2019
 *      Author: marco, edit: jasmin
 */

#include "VelodyneRectifier.h"

namespace fd_scanner
{

VelodyneRectifier::VelodyneRectifier() {
	// TODO Auto-generated constructor stub

}

VelodyneRectifier::~VelodyneRectifier() {
	// TODO Auto-generated destructor stub
}

/**
 * Initialize the velodyne rectifier by setting up subscribers and publishers as well as
 * building LUTs for point cloud generation.
 */
void VelodyneRectifier::init(){
	velodyne_packet_sub_ = nh_.subscribe("velodyne_packets", 10,
			&VelodyneRectifier::velodynePacketCallback, this, ros::TransportHints().tcpNoDelay(true));
	rectified_velodyne_pub_ = nh_.advertise<PointCloud>("rectified_velodyne", 10);
	vertical_correction_[0] = -0.2617993877991494;
	vertical_correction_[1] = 0.017453292519943295;
	vertical_correction_[2] = -0.22689280275926285;
	vertical_correction_[3] = 0.05235987755982989;
	vertical_correction_[4] = -0.19198621771937624;
	vertical_correction_[5] = 0.08726646259971647;
	vertical_correction_[6] = -0.15707963267948966;
	vertical_correction_[7] = 0.12217304763960307;
	vertical_correction_[8] = -0.12217304763960307;
	vertical_correction_[9] = 0.15707963267948966;
	vertical_correction_[10] = -0.08726646259971647;
	vertical_correction_[11] = 0.19198621771937624;
	vertical_correction_[12] = -0.05235987755982989;
	vertical_correction_[13] = 0.22689280275926285;
	vertical_correction_[14] = -0.017453292519943295;
	vertical_correction_[15] = 0.2617993877991494;
	for(int i=0;i<16;i++){
		cos_vertical_correction_[i] = cosf(vertical_correction_[i]);
		sin_vertical_correction_[i] = sinf(vertical_correction_[i]);
	}

	for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
		float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
		cos_rot_table_[rot_index] = cosf(rotation);
		sin_rot_table_[rot_index] = sinf(rotation);
	}
	timing_offsets.resize(12);
	for (size_t i=0; i < timing_offsets.size(); ++i){
		timing_offsets[i].resize(32);
	}
	// constants
	double full_firing_cycle = 55.296 * 1e-6; // seconds
	double single_firing = 2.304 * 1e-6; // seconds
	double dataBlockIndex, dataPointIndex;
	bool dual_mode = false;
	// compute timing offsets
	for (size_t x = 0; x < timing_offsets.size(); ++x){
		for (size_t y = 0; y < timing_offsets[x].size(); ++y){
			if (dual_mode){
				dataBlockIndex = (x - (x % 2)) + (y / 16);
			}
			else{
				dataBlockIndex = (x * 2) + (y / 16);
			}
			dataPointIndex = y % 16;
			//timing_offsets[block][firing]
			timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
		}
	}
}

/**
 * Callback for receiving Velodyne scan packets, building a rectified point cloud from them and publishing it
 */
void VelodyneRectifier::velodynePacketCallback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg){
	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "velodyne";
	pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp );

	for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	{
		unpack(scanMsg->packets[i], msg);

		std::cout << "PACKET NUMBER  " << i << " ... " << std::endl;
	}
	rectified_velodyne_pub_.publish(msg);
}

/**
 * Unpack a packet from the Velodyne scan and extract the single point measurements
 */
void VelodyneRectifier::unpack(const velodyne_msgs::VelodynePacket &pkt, PointCloud::Ptr msg){
	float azimuth;
	float azimuth_diff;
	int raw_azimuth_diff;
	float last_azimuth_diff=0;
	float azimuth_corrected_f;
	int azimuth_corrected;
	float x, y, z;
	float intensity;

	float time_diff_start_to_this_packet = pkt.stamp.toSec();


	const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

	for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

	  std::cout << "... and block number " << block << " of " << BLOCKS_PER_PACKET-1 << "..." << std::endl;

		// ignore packets with mangled or otherwise different contents
		if (UPPER_BANK != raw->blocks[block].header) {
			// Do not flood the log with messages, only issue at most one
			// of these warnings per minute.
			ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
					<< block << " header value is "
					<< raw->blocks[block].header);
			return;                         // bad packet: skip the rest
		}

		// Calculate difference between current and next block's azimuth angle.
		azimuth = (float)(raw->blocks[block].rotation);
		if (block < (BLOCKS_PER_PACKET-1)){
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
				else{
					continue;
				}
			}
			last_azimuth_diff = azimuth_diff;
		}else{
			azimuth_diff = last_azimuth_diff;
		}

		for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){

	    std::cout << "... and firing number " << firing << " of " << VLP16_FIRINGS_PER_BLOCK-1 << "..." << std::endl;

			for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){

			  std::cout << "... and vertical scan number " << dsr << "of " << VLP16_SCANS_PER_FIRING-1 << "..." << std::endl;

				/** Position Calculation */
				union two_bytes tmp;
				tmp.bytes[0] = raw->blocks[block].data[k];
				tmp.bytes[1] = raw->blocks[block].data[k+1];

				/** correct for the laser rotation as a function of timing during the firings **/
				azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
				azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

				// convert polar coordinates to Euclidean XYZ
				float distance = tmp.uint * 0.002;
				//distance += corrections.dist_correction;

        std::cout << "... contains the following distance: " << std::endl;
        std::cout << "distance = " << distance << std::endl;

				float cos_vert_angle = cos_vertical_correction_[dsr];
				float sin_vert_angle = sin_vertical_correction_[dsr];
				float cos_rot_correction = 1.0;
				float sin_rot_correction = 0.0;

				// cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
				// sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
				float cos_rot_angle =
						cos_rot_table_[azimuth_corrected] * cos_rot_correction +
						sin_rot_table_[azimuth_corrected] * sin_rot_correction;
				float sin_rot_angle =
						sin_rot_table_[azimuth_corrected] * cos_rot_correction -
						cos_rot_table_[azimuth_corrected] * sin_rot_correction;

				float horiz_offset = 0.0;
				float vert_offset = 0.0;

				// Compute the distance in the xy plane (w/o accounting for rotation)
				/**the new term of 'vert_offset * sin_vert_angle'
				 * was added to the expression due to the mathemathical
				 * model we used.
				 */
				float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

				// Calculate temporal X, use absolute value.
				float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
				// Calculate temporal Y, use absolute value
				float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
				if (xx < 0) xx=-xx;
				if (yy < 0) yy=-yy;

				float distance_x = distance + 0.0;
				/**the new term of 'vert_offset * sin_vert_angle'
				 * was added to the expression due to the mathemathical
				 * model we used.
				 */
				xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
				x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

				float distance_y = distance + 0.0;
				/**the new term of 'vert_offset * sin_vert_angle'
				 * was added to the expression due to the mathemathical
				 * model we used.
				 */
				xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
				y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

				// Using distance_y is not symmetric, but the velodyne manual
				// does this.
				/**the new term of 'vert_offset * cos_vert_angle'
				 * was added to the expression due to the mathemathical
				 * model we used.
				 */
				z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

				std::cout << "... and contains the following coordinates: " << std::endl;
				std::cout << "x = " << x << std::endl;
				std::cout << "y = " << y << std::endl;
				std::cout << "z = " << z << std::endl;

				/** Use standard ROS coordinate system (right-hand rule) */
				float x_coord = y;
				float y_coord = -x;
				float z_coord = z;



				/** Intensity Calculation */
				float min_intensity = 0.0;
				float max_intensity = 255.0;

				intensity = raw->blocks[block].data[k+2];

				intensity = (intensity < min_intensity) ? min_intensity : intensity;
				intensity = (intensity > max_intensity) ? max_intensity : intensity;

				float time = 0;
				if (timing_offsets.size())
					time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;

//				ros::Time current_time = pkt.stamp + ros::Duration(time);
//
//				geometry_msgs::PointStamped point_in;
//				geometry_msgs::PointStamped transformed_point;
//				point_in.header.frame_id = "velodyne";
//				point_in.header.stamp = ros::Time::now();
//				point_in.point.x = x_coord;
//				point_in.point.y = y_coord;
//				point_in.point.z = z_coord;
//				try {
//					tf_listener_.transformPoint("scanner_link", point_in, transformed_point);
//				} catch (tf::TransformException& ex) {
//					ROS_ERROR("%s", ex.what());
//					return;
//				}
//				msg->points.push_back(pcl::PointXYZ(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z));
				msg->points.push_back(pcl::PointXYZ(x_coord, y_coord, z_coord));
				//data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity, time);

			}
		}
	}

}

}

