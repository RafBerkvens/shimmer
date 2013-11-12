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
    ROS_ASSERT(
        xmlrpc_alignment_matrix[i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(0, 0) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(0, 1) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(0, 2) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(1, 0) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(1, 1) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(1, 2) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(2, 0) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(2, 1) = static_cast<double>(xmlrpc_alignment_matrix[i]);
    ROS_ASSERT(
        xmlrpc_alignment_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    alignment_matrix(2, 2) = static_cast<double>(xmlrpc_alignment_matrix[i]);
  }

  XmlRpc::XmlRpcValue xmlrpc_sensitivity_matrix;
  nh_->getParam(sensor + "/sensitivity_matrix", xmlrpc_sensitivity_matrix);
  ROS_ASSERT(
      xmlrpc_sensitivity_matrix.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < xmlrpc_alignment_matrix.size(); ++i)
  {
    ROS_ASSERT(
        xmlrpc_sensitivity_matrix[i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    sensitivity_matrix(0, 0) =
        static_cast<double>(xmlrpc_sensitivity_matrix[i]);
    sensitivity_matrix(0, 1) = 0.0;
    sensitivity_matrix(0, 2) = 0.0;
    sensitivity_matrix(1, 0) = 0.0;
    ROS_ASSERT(
        xmlrpc_sensitivity_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    sensitivity_matrix(1, 1) =
        static_cast<double>(xmlrpc_sensitivity_matrix[i]);
    sensitivity_matrix(1, 2) = 0.0;
    sensitivity_matrix(2, 0) = 0.0;
    sensitivity_matrix(2, 1) = 0.0;
    ROS_ASSERT(
        xmlrpc_sensitivity_matrix[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    sensitivity_matrix(2, 2) =
        static_cast<double>(xmlrpc_sensitivity_matrix[i]);
  }

  XmlRpc::XmlRpcValue xmlrpc_offset_vector;
  nh_->getParam(sensor + "/sensitivity_matrix", xmlrpc_offset_vector);
  ROS_ASSERT(xmlrpc_offset_vector.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < xmlrpc_offset_vector.size(); ++i)
  {
    ROS_ASSERT(
        xmlrpc_offset_vector[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    offset_vector(0) = static_cast<double>(xmlrpc_offset_vector[i]);
    ROS_ASSERT(
        xmlrpc_offset_vector[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    offset_vector(1) = static_cast<double>(xmlrpc_offset_vector[i]);
    ROS_ASSERT(
        xmlrpc_offset_vector[++i].getType()
            == XmlRpc::XmlRpcValue::TypeDouble);
    offset_vector(2) = static_cast<double>(xmlrpc_offset_vector[i]);
  }
}

CalibrateData::CalibrateData(ros::NodeHandle * nh)
{
  nh_ = nh;
  this->getMatrices("accel",
                    accel_alignment_matrix_,
                    accel_sensitivity_matrix_,
                    accel_offset_vector_);
  this->getMatrices("gyro",
                    gyro_alignment_matrix_,
                    gyro_sensitivity_matrix_,
                    gyro_offset_vector_);
  this->getMatrices("mag",
                    mag_alignment_matrix_,
                    mag_sensitivity_matrix_,
                    mag_offset_vector_);
}

CalibrateData::~CalibrateData()
{
  delete nh_;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shimmer/calibrate_data")
}
