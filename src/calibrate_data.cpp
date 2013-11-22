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

#include "calibrate_data.h"

void CalibrateData::getMatrices(std::string sensor,
                                Eigen::Matrix3d & alignment_matrix,
                                Eigen::Matrix3d & sensitivity_matrix,
                                Eigen::Vector3d & offset_vector)
{
  XmlRpc::XmlRpcValue xmlrpc_alignment_matrix;
  nh_->getParam(sensor + "/alignment_matrix", xmlrpc_alignment_matrix);
  ROS_ASSERT(
      xmlrpc_alignment_matrix.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < xmlrpc_alignment_matrix.size(); ++i)
  {
    ROS_ASSERT(xmlrpc_alignment_matrix[i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(0, 0) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(0, 1) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(0, 2) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(1, 0) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(1, 1) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(1, 2) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(2, 0) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(2, 1) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(xmlrpc_alignment_matrix[++i].getType()
        == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(2, 2) = static_cast<double>(xmlrpc_alignment_matrix[i]);
  }

  XmlRpc::XmlRpcValue xmlrpc_sensitivity_matrix;
  nh_->getParam(sensor + "/sensitivity_matrix", xmlrpc_sensitivity_matrix);
  ROS_ASSERT(
      xmlrpc_sensitivity_matrix.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < xmlrpc_sensitivity_matrix.size(); ++i)
  {
    if (xmlrpc_sensitivity_matrix[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ASSERT(xmlrpc_sensitivity_matrix[i].getType()
          == XmlRpc::XmlRpcValue::TypeInt);
      sensitivity_matrix(0, 0) =
          double(static_cast<int>(xmlrpc_sensitivity_matrix[i]));
      sensitivity_matrix(0, 1) = 0.0;
      sensitivity_matrix(0, 2) = 0.0;
      sensitivity_matrix(1, 0) = 0.0;
      ROS_ASSERT(xmlrpc_sensitivity_matrix[++i].getType()
          == XmlRpc::XmlRpcValue::TypeInt);
      sensitivity_matrix(1, 1) =
          double(static_cast<int>(xmlrpc_sensitivity_matrix[i]));
      sensitivity_matrix(1, 2) = 0.0;
      sensitivity_matrix(2, 0) = 0.0;
      sensitivity_matrix(2, 1) = 0.0;
      ROS_ASSERT(xmlrpc_sensitivity_matrix[++i].getType()
          == XmlRpc::XmlRpcValue::TypeInt);
      sensitivity_matrix(2, 2) =
          double(static_cast<int>(xmlrpc_sensitivity_matrix[i]));
    }
    else
    {
      ROS_ASSERT(xmlrpc_sensitivity_matrix[i].getType()
          == XmlRpc::XmlRpcValue::TypeDouble);
      sensitivity_matrix(0, 0) =
          static_cast<double>(xmlrpc_sensitivity_matrix[i]);
      sensitivity_matrix(0, 1) = 0.0;
      sensitivity_matrix(0, 2) = 0.0;
      sensitivity_matrix(1, 0) = 0.0;
      ROS_ASSERT(xmlrpc_sensitivity_matrix[++i].getType()
          == XmlRpc::XmlRpcValue::TypeDouble);
      sensitivity_matrix(1, 1) =
          static_cast<double>(xmlrpc_sensitivity_matrix[i]);
      sensitivity_matrix(1, 2) = 0.0;
      sensitivity_matrix(2, 0) = 0.0;
      sensitivity_matrix(2, 1) = 0.0;
      ROS_ASSERT(xmlrpc_sensitivity_matrix[++i].getType()
          == XmlRpc::XmlRpcValue::TypeDouble);
      sensitivity_matrix(2, 2) =
          static_cast<double>(xmlrpc_sensitivity_matrix[i]);
    }
  }

  XmlRpc::XmlRpcValue xmlrpc_offset_vector;
  nh_->getParam(sensor + "/offset_vector", xmlrpc_offset_vector);
  ROS_ASSERT(xmlrpc_offset_vector.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < xmlrpc_offset_vector.size(); ++i)
  {
    ROS_ASSERT(
        xmlrpc_offset_vector[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    offset_vector(0) = double(static_cast<int>(xmlrpc_offset_vector[i]));
    ROS_ASSERT(
        xmlrpc_offset_vector[++i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    offset_vector(1) = double(static_cast<int>(xmlrpc_offset_vector[i]));
    ROS_ASSERT(
        xmlrpc_offset_vector[++i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    offset_vector(2) = double(static_cast<int>(xmlrpc_offset_vector[i]));
  }
}

Eigen::Vector3d CalibrateData::calibrate(Eigen::Vector3d data,
                                         Eigen::Matrix3d alignment_matrix,
                                         Eigen::Matrix3d sensitivity_matrix,
                                         Eigen::Vector3d offset_vector)
{
  Eigen::Vector3d calibrated_data;
  calibrated_data = alignment_matrix.inverse() *
      sensitivity_matrix.inverse() *
      (data - offset_vector);
  return calibrated_data;
}

CalibrateData::CalibrateData(ros::NodeHandle * nh)
{
  nh_ = nh;
  try
  {
    this->getMatrices("/shimmer/accel",
                      accel_alignment_matrix_,
                      accel_sensitivity_matrix_,
                      accel_offset_vector_);
    this->getMatrices("/shimmer/gyro",
                      gyro_alignment_matrix_,
                      gyro_sensitivity_matrix_,
                      gyro_offset_vector_);
    this->getMatrices("/shimmer/mag",
                      mag_alignment_matrix_,
                      mag_sensitivity_matrix_,
                      mag_offset_vector_);
  }
  catch (XmlRpc::XmlRpcException &e)
  {
    ROS_FATAL_STREAM("XmlRpc error when extracting parameters: " <<
                     e.getMessage() << ". Code: " << e.getCode());
    ros::shutdown();
  }

  imu_pub_ = nh_->advertise<sensor_msgs::Imu>("shimmer/imu", 1);
  mag_pub_ = nh_->advertise<sensor_msgs::MagneticField>("shimmer/mag", 1);
  heading_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("shimmer/heading",
                                                            1);
}

CalibrateData::~CalibrateData()
{
  //delete nh_;
}

void CalibrateData::callbackAccelGyro(const sensor_msgs::ImuConstPtr & msg)
{
  Eigen::Vector3d accel_raw;
  Eigen::Vector3d accel_cal;
  Eigen::Vector3d gyro_raw;
  Eigen::Vector3d gyro_cal;

  accel_raw(0) = msg->linear_acceleration.x;
  accel_raw(1) = msg->linear_acceleration.y;
  accel_raw(2) = msg->linear_acceleration.z;
  gyro_raw(0) = msg->angular_velocity.x;
  gyro_raw(1) = msg->angular_velocity.y;
  gyro_raw(2) = msg->angular_velocity.z;

  accel_cal = calibrate(accel_raw,
                        accel_alignment_matrix_,
                        accel_sensitivity_matrix_,
                        accel_offset_vector_);
  gyro_cal = calibrate(gyro_raw,
                       gyro_alignment_matrix_,
                       gyro_sensitivity_matrix_,
                       gyro_offset_vector_);

  sensor_msgs::Imu imu_msg = *msg;
  imu_msg.linear_acceleration.x = accel_cal(0);
  imu_msg.linear_acceleration.y = accel_cal(1);
  imu_msg.linear_acceleration.z = accel_cal(2);
  imu_msg.angular_velocity.x = gyro_cal(0) * M_PI / 180.0;
  imu_msg.angular_velocity.y = gyro_cal(1) * M_PI / 180.0;
  imu_msg.angular_velocity.z = gyro_cal(2) * M_PI / 180.0;

  imu_pub_.publish(imu_msg);
}

void CalibrateData::callbackMag(const sensor_msgs::MagneticFieldConstPtr & msg)
{
  Eigen::Vector3d mag_raw;
  Eigen::Vector3d mag_cal;

  mag_raw(0) = msg->magnetic_field.x;
  mag_raw(1) = msg->magnetic_field.y;
  mag_raw(2) = msg->magnetic_field.z;

  mag_cal = calibrate(mag_raw,
                      mag_alignment_matrix_,
                      mag_sensitivity_matrix_,
                      mag_offset_vector_);

  double heading = -1.0 * atan2(double(mag_cal(1)), double(mag_cal(0)));
  geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(heading);

  sensor_msgs::MagneticField mag_msg = *msg;
  mag_msg.magnetic_field.x = mag_cal(0);
  mag_msg.magnetic_field.y = mag_cal(1);
  mag_msg.magnetic_field.z = mag_cal(2);

  geometry_msgs::PoseStamped heading_msg;
  heading_msg.header = msg->header;
  heading_msg.pose.position.x = 0.0;
  heading_msg.pose.position.y = 0.0;
  heading_msg.pose.position.z = 0.0;
  heading_msg.pose.orientation = orientation;

  mag_pub_.publish(mag_msg);
  heading_pub_.publish(heading_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibrate_data");
  ros::NodeHandle nh;

  CalibrateData calibrateData(&nh);
  ros::Subscriber imu_sub = nh.subscribe("shimmer/raw/imu", 1,
                                         &CalibrateData::callbackAccelGyro,
                                         &calibrateData);
  ros::Subscriber mag_sub = nh.subscribe("shimmer/raw/mag", 1,
                                         &CalibrateData::callbackMag,
                                         &calibrateData);

  ros::spin();
}
