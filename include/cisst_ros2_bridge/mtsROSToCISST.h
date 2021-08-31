/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsROSToCISST_h
#define _mtsROSToCISST_h

// cisst include
#include <cisstMultiTask/mtsManagerLocal.h>

#include <cisstVector/vctDynamicVectorTypes.h>

#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmCartesianImpedanceGains.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmVelocityCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmOperatingState.h>

// ros include
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

// non standard messages
#include <cisst_msgs/msg/double_vec.hpp>
#include <cisst_msgs/msg/interval_statistics.hpp>
#include <cisst_msgs/msg/cartesian_impedance_gains.hpp>
#include <cisst_msgs/srv/query_forward_kinematics.hpp>

// helper functions
template <typename _cisstFrame>
void mtsROSTransformToCISST(const geometry_msgs::msg::Transform & rosTransform, _cisstFrame & cisstFrame)
{
    cisstFrame.Translation().X() = rosTransform.translation.x;
    cisstFrame.Translation().Y() = rosTransform.translation.y;
    cisstFrame.Translation().Z() = rosTransform.translation.z;
    vctQuatRot3 quat;
    quat.X() = rosTransform.rotation.x;
    quat.Y() = rosTransform.rotation.y;
    quat.Z() = rosTransform.rotation.z;
    quat.W() = rosTransform.rotation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstFrame.Rotation().Assign(rotation);
}

template <typename _cisstFrame>
void mtsROSPoseToCISST(const geometry_msgs::msg::Pose & rosPose, _cisstFrame & cisstFrame)
{
    cisstFrame.Translation().X() = rosPose.position.x;
    cisstFrame.Translation().Y() = rosPose.position.y;
    cisstFrame.Translation().Z() = rosPose.position.z;
    vctQuatRot3 quat;
    quat.X() = rosPose.orientation.x;
    quat.Y() = rosPose.orientation.y;
    quat.Z() = rosPose.orientation.z;
    quat.W() = rosPose.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstFrame.Rotation().Assign(rotation);
}

template <typename _cisstType>
void mtsROSToCISSTNoHeader(_cisstType & cisstData)
{
    const double cisstNow = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
    cisstData.SetTimestamp(cisstNow);
    // always set as valid for now
    cisstData.SetValid(true);
}

template <typename _rosType, typename _cisstType>
void mtsROSToCISSTHeader(const _rosType & rosData, _cisstType & cisstData)
{
    const double cisstNow = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
    // first check that header.stamp is not zero
    rclcpp::Time time(rosData.header.stamp);
    if (time == rclcpp::Time(0, 0)) {
        cisstData.SetTimestamp(cisstNow);
    } else {
        rclcpp::Clock clock;
        const double ageInSeconds = (clock.now() - rosData.header.stamp).seconds();
        if (ageInSeconds > 0.0) {
            cisstData.SetTimestamp(cisstNow - ageInSeconds);
        } else {
            cisstData.SetTimestamp(cisstNow);
        }
    }
    // always set as valid for now
    cisstData.SetValid(true);
}

// std_msgs
void mtsROSToCISST(const std_msgs::msg::Float32 & rosData, double & cisstData);
void mtsROSToCISST(const std_msgs::msg::Float64 & rosData, double & cisstData);
void mtsROSToCISST(const std_msgs::msg::Int32 & rosData, int & cisstData);
void mtsROSToCISST(const std_msgs::msg::Bool & rosData, bool & cisstData);
void mtsROSToCISST(const std_msgs::msg::String & rosData, std::string & cisstData);
void mtsROSToCISST(const std_msgs::msg::String & rosData, mtsMessage & cisstData);
void mtsROSToCISST(const std_msgs::msg::Float64MultiArray & rosData, vctDoubleVec & cisstData);

// geometry_msgs
void mtsROSToCISST(const geometry_msgs::msg::Vector3 & rosData, vct3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Quaternion & rosData, vctMatRot3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, prmForceCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, prmForceCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, mtsDoubleVec & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, mtsDoubleVec & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Twist & rosData, prmVelocityCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TwistStamped & rosData, prmVelocityCartesianSet & cisstData);

// sensor_msgs
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmPositionJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmForceTorqueJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmVelocityJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmStateJoint & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::Joy & rosData, prmEventButton & cisstData);

// diagnostic_msgs
void mtsROSToCISST(const diagnostic_msgs::msg::KeyValue & rosData, prmKeyValue & cisstData);

// cisst_msgs
void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, prmPositionJointSet & cisstData);
void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, vctDoubleVec & cisstData);
void mtsROSToCISST(const cisst_msgs::msg::CartesianImpedanceGains & rosData, prmCartesianImpedanceGains & cisstData);
void mtsROSToCISST(const cisst_msgs::msg::IntervalStatistics & rosData, mtsIntervalStatistics & cisstData);
void mtsROSToCISST(const cisst_msgs::srv::QueryForwardKinematics::Request & rosData,
                   vctDoubleVec & cisstData);

#endif // _mtsROSToCISST_h
