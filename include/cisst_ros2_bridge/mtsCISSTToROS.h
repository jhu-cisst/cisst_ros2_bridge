/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsCISSTToROS_h
#define _mtsCISSTToROS_h

// cisst include
#include <cisstVector/vctDynamicVectorTypes.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianArrayGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmCartesianImpedanceGains.h>
#include <cisstParameterTypes/prmInputData.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmOperatingState.h>

// ros include
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>

// non standard messages
#include <cisst_msgs/msg/double_vec.hpp>
#include <cisst_msgs/msg/cartesian_impedance_gains.hpp>
#include <cisst_msgs/msg/interval_statistics.hpp>
#include <cisst_msgs/msg/bool_stamped.hpp>
#include <cisst_msgs/srv/query_forward_kinematics.hpp>

// helper functions
template <typename _cisstFrame, typename _rosPose>
void mtsCISSTToROSPose(const _cisstFrame & cisstFrame, _rosPose & rosPose)
{
    vctQuatRot3 quat(cisstFrame.Rotation(), VCT_NORMALIZE);
    rosPose.orientation.x = quat.X();
    rosPose.orientation.y = quat.Y();
    rosPose.orientation.z = quat.Z();
    rosPose.orientation.w = quat.W();
    rosPose.position.x = cisstFrame.Translation().X();
    rosPose.position.y = cisstFrame.Translation().Y();
    rosPose.position.z = cisstFrame.Translation().Z();
}

template <typename _cisstFrame, typename _rosTransform>
void mtsCISSTToROSTransform(const _cisstFrame & cisstFrame, _rosTransform & rosTransform)
{
    vctQuatRot3 quat(cisstFrame.Rotation(), VCT_NORMALIZE);
    rosTransform.rotation.x = quat.X();
    rosTransform.rotation.y = quat.Y();
    rosTransform.rotation.z = quat.Z();
    rosTransform.rotation.w = quat.W();
    rosTransform.translation.x = cisstFrame.Translation().X();
    rosTransform.translation.y = cisstFrame.Translation().Y();
    rosTransform.translation.z = cisstFrame.Translation().Z();
}

template <typename _cisstType, typename _rosType>
bool mtsCISSTToROSHeader(const _cisstType & cisstData, _rosType & rosData,
                         std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (!cisstData.Valid()) {
        return false;
    }
    try {
        const double cisstDataTime = cisstData.Timestamp();
        if (cisstDataTime > 0.0) {
            const double age =
                mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime()
                - cisstDataTime;
            if (age > 0.0) {
                rosData.header.stamp = node->get_clock()->now() - rclcpp::Duration::from_seconds(age);
            } else {
                rosData.header.stamp = node->get_clock()->now();
            }
        } else {
            rosData.header.stamp = node->get_clock()->now();
        }
    } catch (std::exception & e) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROSHeader caught exception \""
                          << e.what()
                          << "\"while computing timestamp for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    return true;
}

template <typename _rosType>
bool mtsCISSTToROSHeader(_rosType & rosData, std::shared_ptr<rclcpp::Node> node,
                         const std::string & CMN_UNUSED(debugInfo))
{
    rclcpp::Clock clock;
    rosData.header.stamp = node->get_clock()->now();
    return true;
}

// std_msgs
bool mtsCISSTToROS(const double & cisstData, std_msgs::msg::Float32 & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const double & cisstData, std_msgs::msg::Float64 & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const int & cisstData, std_msgs::msg::Int32 & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const bool & cisstData, std_msgs::msg::Bool & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const bool & cisstData, cisst_msgs::msg::BoolStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const bool & cisstData, sensor_msgs::msg::Joy & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const std::string & cisstData, std_msgs::msg::String & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsMessage & cisstData, std_msgs::msg::String & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::msg::Bool & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmEventButton & cisstData, cisst_msgs::msg::BoolStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmEventButton & cisstData, sensor_msgs::msg::Joy & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleVec & cisstData, std_msgs::msg::Float64MultiArray & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleMat & cisstData, std_msgs::msg::Float64MultiArray & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);

// geometry_msgs
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::Transform & rosData
                   , std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::PoseStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianArrayGet & cisstData, geometry_msgs::msg::PoseArray & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::msg::Vector3 & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::msg::Quaternion & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::msg::QuaternionStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::Wrench & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::Vector3Stamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::msg::Twist & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::msg::TwistStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::msg::Wrench & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);

// sensor_msgs
bool mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionJointSet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::msg::PointCloud & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const std::vector<vct3> & cisstData, sensor_msgs::msg::PointCloud & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmInputData & cisstData, sensor_msgs::msg::Joy & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);

// diagnostic_msgs
bool mtsCISSTToROS(const prmKeyValue & cisstData, diagnostic_msgs::msg::KeyValue & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);

// std_srvs
bool mtsCISSTToROS(const bool & cisstData, std_srvs::srv::Trigger::Response & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const std::string & cisstData, std_srvs::srv::Trigger::Response & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);

// cisst_msgs
bool mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::msg::DoubleVec & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::msg::DoubleVec & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const prmCartesianImpedanceGains & cisstData, cisst_msgs::msg::CartesianImpedanceGains & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsIntervalStatistics & cisstData, cisst_msgs::msg::IntervalStatistics & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm4x4 & cisstData, cisst_msgs::srv::QueryForwardKinematics::Response & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo);

#endif // _mtsCISSTToROS_h
