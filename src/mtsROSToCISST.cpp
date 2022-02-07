/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <cisst_ros2_bridge/mtsROSToCISST.h>

void mtsROSToCISST(const std_msgs::msg::Float32 & rosData, double & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Float64 & rosData, double & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Int32 & rosData, int & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Bool & rosData, bool & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::String & rosData, std::string & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::String & rosData, mtsMessage & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData.Message = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Float64MultiArray & rosData, vctDoubleVec & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    const size_t size = rosData.data.size();
    if (size != 0) {
        cisstData.SetSize(size);
        std::copy(rosData.data.begin(), rosData.data.end(),
                  cisstData.begin());
    }
}

void mtsROSToCISST(const std_msgs::msg::Float64MultiArray & rosData, vctDoubleMat & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    if (rosData.layout.dim.size() != 2) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): incoming array dimension is not 2");
    }
    // assuming rows/cols for data storage.  In mtsCISSTToROS we
    // labelled the dimensions using "rows" and "cols" so we're making
    // sure names are the same.  This might be annoying for data
    // coming from other packages and we might have to change this
    // check.  We could also add support for cols/rows storage order
    if ((rosData.layout.dim[0].label != "rows")
        || (rosData.layout.dim[1].label != "cols")) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): dimensions must be labelled \"rows\" and \"cols\"");
    }
    // now check the strides and sizes
    const size_t rows = rosData.layout.dim[0].size;
    const size_t cols = rosData.layout.dim[1].size;
    if (rosData.layout.dim[0].stride != 1) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): dim[0].stride must be equal to 1");
    }
    if (rosData.layout.dim[1].stride != rows) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): dim[1].stride must be equal to number of rows");
    }
    // now allocate and copy data
    cisstData.SetSize(rows, cols);
    std::copy(rosData.data.begin(), rosData.data.end(),
              cisstData.begin());
}

void mtsROSToCISST(const geometry_msgs::msg::Vector3 & rosData, vct3 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData[0] = rosData.x;
    cisstData[1] = rosData.y;
    cisstData[2] = rosData.z;
}

void mtsROSToCISST(const geometry_msgs::msg::Quaternion & rosData, vctMatRot3 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    vctQuatRot3 quat;
    quat.X() = rosData.x;
    quat.Y() = rosData.y;
    quat.Z() = rosData.z;
    quat.W() = rosData.w;
    cisstData.FromNormalized(quat);
}


void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData.Goal());
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.pose, cisstData, node);
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.pose, cisstData, node);
}

void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.transform, cisstData.Position(), node);
}

void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.transform, cisstData.Goal(), node);
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm3 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm4x4 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm3 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm4x4 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, mtsFrm4x4 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, prmPositionCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSTransformToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm3 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm4x4 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, mtsFrm4x4 & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, mtsFrm4x4 & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSTransformToCISST(rosData.transform, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, prmForceCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    vctFixedSizeVector<double, 6>
        vctFT(rosData.force.x, rosData.force.y, rosData.force.z,
              rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
}

void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, prmForceCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.wrench, cisstData, node);
}

void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, prmForceCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    vctFixedSizeVector<double, 6>
        vctFT(rosData.force.x, rosData.force.y, rosData.force.z,
              rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
}

void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, prmForceCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.wrench, cisstData, node);
}

void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, mtsDoubleVec & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    cisstData.SetSize(6);
    cisstData.Element(0) = rosData.force.x;
    cisstData.Element(1) = rosData.force.y;
    cisstData.Element(2) = rosData.force.z;
    cisstData.Element(3) = rosData.torque.x;
    cisstData.Element(4) = rosData.torque.y;
    cisstData.Element(5) = rosData.torque.z;
}

void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, mtsDoubleVec & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.wrench, cisstData, node);
}

void mtsROSToCISST(const geometry_msgs::msg::Twist & rosData, prmVelocityCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    cisstData.SetVelocityLinear(vct3(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetVelocityAngular(vct3(rosData.angular.x, rosData.angular.y, rosData.angular.z));
}

void mtsROSToCISST(const geometry_msgs::msg::TwistStamped & rosData, prmVelocityCartesianGet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.twist, cisstData, node);
}

void mtsROSToCISST(const geometry_msgs::msg::Twist & rosData, prmVelocityCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    mtsROSToCISSTNoHeader(cisstData);
    cisstData.SetVelocity(vct3(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetAngularVelocity(vct3(rosData.angular.x, rosData.angular.y, rosData.angular.z));
}

void mtsROSToCISST(const geometry_msgs::msg::TwistStamped & rosData, prmVelocityCartesianSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    mtsROSToCISST(rosData.twist, cisstData, node);
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmPositionJointSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    cisstData.Goal().SetSize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmForceTorqueJointSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    cisstData.ForceTorque().SetSize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.ForceTorque().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmVelocityJointSet & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    cisstData.Goal().SetSize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmStateJoint & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    cisstData.Name().SetSize(rosData.name.size());
    std::copy(rosData.name.begin(), rosData.name.end(),
              cisstData.Name().begin());
    cisstData.Position().SetSize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Position().begin());
    cisstData.Velocity().SetSize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Velocity().begin());
    cisstData.Effort().SetSize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.Effort().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::Joy & rosData, prmEventButton & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    if (rosData.buttons.size() < 1) {
        cisstData.Type() = prmEventButton::UNDEFINED;
        return;
    }
    if (rosData.buttons[0] == 1) {
        cisstData.Type() = prmEventButton::PRESSED;
    } else if (rosData.buttons[0] == 0) {
        cisstData.Type() = prmEventButton::RELEASED;
    } else if (rosData.buttons[0] == 2) {
        cisstData.Type() = prmEventButton::CLICKED;
    } else {
        cisstData.Type() = prmEventButton::UNDEFINED;
    }
}

void mtsROSToCISST(const sensor_msgs::msg::Joy & rosData, prmInputData & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    cisstData.AnalogInputs().SetSize(rosData.axes.size());
    cisstData.DigitalInputs().SetSize(rosData.buttons.size());
    std::copy(rosData.axes.begin(), rosData.axes.end(),
              cisstData.AnalogInputs().begin());
    std::copy(rosData.buttons.begin(), rosData.buttons.end(),
              cisstData.DigitalInputs().begin());
}

void mtsROSToCISST(const diagnostic_msgs::msg::KeyValue & rosData, prmKeyValue & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData.Key = rosData.key;
    cisstData.Value = rosData.value;
}

void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, prmPositionJointSet & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    const size_t size = rosData.data.size();
    cisstData.Goal().resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Goal().Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, vctDoubleVec & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    const size_t size = rosData.data.size();
    cisstData.resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const cisst_msgs::msg::CartesianImpedanceGains & rosData,
                   prmCartesianImpedanceGains & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    // vf pos/rot
    mtsROSToCISST(rosData.force_position,
                  cisstData.ForcePosition(), node);
    mtsROSToCISST(rosData.force_orientation,
                  cisstData.ForceOrientation(), node);
    mtsROSToCISST(rosData.torque_orientation,
                  cisstData.TorqueOrientation(), node);

    // force gains
    mtsROSToCISST(rosData.pos_deadband_pos,
                  cisstData.PositionDeadbandPos(), node);
    mtsROSToCISST(rosData.pos_deadband_neg,
                  cisstData.PositionDeadbandNeg(), node);
    mtsROSToCISST(rosData.pos_stiff_pos,
                  cisstData.PositionStiffnessPos(), node);
    mtsROSToCISST(rosData.pos_stiff_neg,
                  cisstData.PositionStiffnessNeg(), node);
    mtsROSToCISST(rosData.pos_damping_pos,
                  cisstData.PositionDampingPos(), node);
    mtsROSToCISST(rosData.pos_damping_neg,
                  cisstData.PositionDampingNeg(), node);
    mtsROSToCISST(rosData.force_bias_pos,
                  cisstData.ForceBiasPos(), node);
    mtsROSToCISST(rosData.force_bias_neg,
                  cisstData.ForceBiasNeg(), node);

    // torque gains
    mtsROSToCISST(rosData.ori_deadband_pos,
                  cisstData.OrientationDeadbandPos(), node);
    mtsROSToCISST(rosData.ori_deadband_neg,
                  cisstData.OrientationDeadbandNeg(), node);
    mtsROSToCISST(rosData.ori_stiff_pos,
                  cisstData.OrientationStiffnessPos(), node);
    mtsROSToCISST(rosData.ori_stiff_neg,
                  cisstData.OrientationStiffnessNeg(), node);
    mtsROSToCISST(rosData.ori_damping_pos,
                  cisstData.OrientationDampingPos(), node);
    mtsROSToCISST(rosData.ori_damping_neg,
                  cisstData.OrientationDampingNeg(), node);
    mtsROSToCISST(rosData.torque_bias_pos,
                  cisstData.TorqueBiasPos(), node);
    mtsROSToCISST(rosData.torque_bias_neg,
                  cisstData.TorqueBiasNeg(), node);
}

void mtsROSToCISST(const cisst_msgs::msg::IntervalStatistics & rosData, mtsIntervalStatistics & cisstData,
                   std::shared_ptr<rclcpp::Node> node)
{
    mtsROSToCISSTHeader(rosData, cisstData, node);
    cisstData.SetFromExisting(rosData.period_avg,
                              rosData.period_std_dev,
                              rosData.period_min,
                              rosData.period_max,
                              rosData.compute_time_avg,
                              rosData.compute_time_std_dev,
                              rosData.compute_time_min,
                              rosData.compute_time_max,
                              rosData.number_of_samples,
                              rosData.number_of_overruns,
                              rosData.statistics_interval);
}

void mtsROSToCISST(const cisst_msgs::srv::QueryForwardKinematics::Request & rosData,
                   vctDoubleVec & cisstData,
                   std::shared_ptr<rclcpp::Node>)
{
    cisstData.SetSize(rosData.jp.position.size());
    std::copy(rosData.jp.position.begin(), rosData.jp.position.end(),
              cisstData.begin());
}
