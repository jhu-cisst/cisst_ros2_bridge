/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <cisst_ros2_bridge/mtsROSToCISST.h>

void mtsROSToCISST(const std_msgs::msg::Float32 & rosData, double & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Float64 & rosData, double & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Int32 & rosData, int & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Bool & rosData, bool & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::String & rosData, std::string & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::String & rosData, mtsMessage & cisstData)
{
    cisstData.Message = rosData.data;
}

void mtsROSToCISST(const std_msgs::msg::Float64MultiArray & rosData, vctDoubleVec & cisstData)
{
    const size_t size = rosData.data.size();
    if (size != 0) {
        cisstData.SetSize(size);
        std::copy(rosData.data.begin(), rosData.data.end(),
                  cisstData.begin());
    }
}

void mtsROSToCISST(const geometry_msgs::msg::Vector3 & rosData, vct3 & cisstData)
{
    cisstData[0] = rosData.x;
    cisstData[1] = rosData.y;
    cisstData[2] = rosData.z;
}

void mtsROSToCISST(const geometry_msgs::msg::Quaternion & rosData, vctMatRot3 & cisstData)
{
    vctQuatRot3 quat;
    quat.X() = rosData.x;
    quat.Y() = rosData.y;
    quat.Z() = rosData.z;
    quat.W() = rosData.w;
    cisstData.FromNormalized(quat);
}


void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianGet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData.Goal());
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianGet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianGet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.transform, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.transform, cisstData.Goal());
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm3 & cisstData)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm4x4 & cisstData)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm3 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm4x4 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, mtsFrm4x4 & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, prmPositionCartesianGet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSTransformToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm3 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm4x4 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, mtsFrm4x4 & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, mtsFrm4x4 & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSTransformToCISST(rosData.transform, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, prmForceCartesianSet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    vctFixedSizeVector<double, 6>
        vctFT(rosData.force.x, rosData.force.y, rosData.force.z,
              rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
}

void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, prmForceCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, mtsDoubleVec & cisstData)
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

void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, mtsDoubleVec & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const geometry_msgs::msg::Twist & rosData, prmVelocityCartesianSet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    cisstData.SetVelocity(vct3(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetAngularVelocity(vct3(rosData.angular.x, rosData.angular.y, rosData.angular.z));
}

void mtsROSToCISST(const geometry_msgs::msg::TwistStamped & rosData, prmVelocityCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    rclcpp::Time time(rosData.header.stamp);
    cisstData.SetTimestamp(time.seconds());
    mtsROSToCISST(rosData.twist, cisstData);
}


void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmPositionJointSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.Goal().SetSize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmForceTorqueJointSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.ForceTorque().SetSize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.ForceTorque().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmVelocityJointSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.Goal().SetSize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmStateJoint & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
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

void mtsROSToCISST(const sensor_msgs::msg::Joy & rosData, prmEventButton & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
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

void mtsROSToCISST(const diagnostic_msgs::msg::KeyValue & rosData, prmKeyValue & cisstData)
{
    cisstData.Key = rosData.key;
    cisstData.Value = rosData.value;
}

void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, prmPositionJointSet & cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.Goal().resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Goal().Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, vctDoubleVec & cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const cisst_msgs::msg::CartesianImpedanceGains & rosData,
                   prmCartesianImpedanceGains & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    // vf pos/rot
    mtsROSToCISST(rosData.force_position,
                  cisstData.ForcePosition());
    mtsROSToCISST(rosData.force_orientation,
                  cisstData.ForceOrientation());
    mtsROSToCISST(rosData.torque_orientation,
                  cisstData.TorqueOrientation());

    // force gains
    mtsROSToCISST(rosData.pos_deadband_pos,
                  cisstData.PositionDeadbandPos());
    mtsROSToCISST(rosData.pos_deadband_neg,
                  cisstData.PositionDeadbandNeg());
    mtsROSToCISST(rosData.pos_stiff_pos,
                  cisstData.PositionStiffnessPos());
    mtsROSToCISST(rosData.pos_stiff_neg,
                  cisstData.PositionStiffnessNeg());
    mtsROSToCISST(rosData.pos_damping_pos,
                  cisstData.PositionDampingPos());
    mtsROSToCISST(rosData.pos_damping_neg,
                  cisstData.PositionDampingNeg());
    mtsROSToCISST(rosData.force_bias_pos,
                  cisstData.ForceBiasPos());
    mtsROSToCISST(rosData.force_bias_neg,
                  cisstData.ForceBiasNeg());

    // torque gains
    mtsROSToCISST(rosData.ori_deadband_pos,
                  cisstData.OrientationDeadbandPos());
    mtsROSToCISST(rosData.ori_deadband_neg,
                  cisstData.OrientationDeadbandNeg());
    mtsROSToCISST(rosData.ori_stiff_pos,
                  cisstData.OrientationStiffnessPos());
    mtsROSToCISST(rosData.ori_stiff_neg,
                  cisstData.OrientationStiffnessNeg());
    mtsROSToCISST(rosData.ori_damping_pos,
                  cisstData.OrientationDampingPos());
    mtsROSToCISST(rosData.ori_damping_neg,
                  cisstData.OrientationDampingNeg());
    mtsROSToCISST(rosData.torque_bias_pos,
                  cisstData.TorqueBiasPos());
    mtsROSToCISST(rosData.torque_bias_neg,
                  cisstData.TorqueBiasNeg());
}

void mtsROSToCISST(const cisst_msgs::msg::IntervalStatistics & rosData, mtsIntervalStatistics & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
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
                   vctDoubleVec & cisstData)
{
    cisstData.SetSize(rosData.jp.position.size());
    std::copy(rosData.jp.position.begin(), rosData.jp.position.end(),
              cisstData.begin());
}