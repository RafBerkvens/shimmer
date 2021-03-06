/*
 Listens to Shimmer data and applies calibration based on parameters.
 Copyright (C) 2013 Rafael Berkvens rafael.berkvens@uantwerpen.be

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CALIBRATE_DATA_H_
#define CALIBRATE_DATA_H_

#include <string>
#include <math.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

class CalibrateData
{
  Eigen::Matrix3d accel_alignment_matrix_, accel_sensitivity_matrix_;
  Eigen::Vector3d accel_offset_vector_;
  Eigen::Matrix3d gyro_alignment_matrix_, gyro_sensitivity_matrix_;
  Eigen::Vector3d gyro_offset_vector_;
  Eigen::Matrix3d mag_alignment_matrix_, mag_sensitivity_matrix_;
  Eigen::Vector3d mag_offset_vector_;

  ros::NodeHandle * nh_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher heading_pub_;

  /**
   * Values needed to compute the Exponential Moving Average (EMA) of the
   * heading. For more information see Wikipedia:
   * http://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
   *
   * TODO: add ROS parameters to edit values.
   */
  double ema_alpha_;
  double ema_heading_;

  /** Helper method to get matrices out of structured yaml files.
   * Getting lists out of yaml parameter files is non-trivial and involves
   * the XmlRpc::XmlRpcValue class. For more information, see:
   * http://wiki.ros.org/roscpp/Overview/Parameter%20Server
   *
   * @param sensor: indicate the accelerometer, gyroscope or compass sensor by
   * respectively "accel", "gyro", or "mag".
   * @param alignment_matrix: used to store the alignment matrix in; content
   * will be overwritten.
   * @param sensitivity_matrix: used to store the sensitivity matrix in; content
   * will be overwritten.
   * @param offset_vector: used to store the offset vector in; content will be
   * overwritten.
   */
  void getMatrices(std::string sensor,
                   Eigen::Matrix3d & alignment_matrix,
                   Eigen::Matrix3d & sensitivity_matrix,
                   Eigen::Vector3d & offset_vector);

  Eigen::Vector3d calibrate(Eigen::Vector3d data,
                            Eigen::Matrix3d alignment_matrix,
                            Eigen::Matrix3d sensitivity_matrix,
                            Eigen::Vector3d offset_vector);

public:
  CalibrateData(ros::NodeHandle * nh);
  virtual ~CalibrateData();

  void callbackAccelGyro(const sensor_msgs::ImuConstPtr & msg);
  void callbackMag(const sensor_msgs::MagneticFieldConstPtr & msg);
};

#endif /* CALIBRATE_DATA_H_ */
