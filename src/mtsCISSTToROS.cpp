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

#include <cisst_ros2_bridge/mtsCISSTToROS.h>

bool mtsCISSTToROS(const double & cisstData, std_msgs::msg::Float32 & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const double & cisstData, std_msgs::msg::Float64 & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const int & cisstData, std_msgs::msg::Int32 & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const bool & cisstData, std_msgs::msg::Bool & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const bool & cisstData, cisst_msgs::msg::BoolStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(rosData, node, debugInfo)) {
        rosData.data = cisstData;
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const bool & cisstData, sensor_msgs::msg::Joy & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(rosData, node, debugInfo)) {
        rosData.axes.resize(0);
        rosData.buttons.resize(1);
        rosData.buttons[0] = cisstData;
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const std::string & cisstData, std_msgs::msg::String & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const mtsMessage & cisstData, std_msgs::msg::String & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.data = cisstData.Message;
    return true;
}

bool mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::msg::Bool & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    if (cisstData.Valid()) {
        if (cisstData.Type() == prmEventButton::PRESSED) {
            rosData.data = true;
        } else if (cisstData.Type() == prmEventButton::RELEASED) {
            rosData.data = false;
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmEventButton & cisstData, cisst_msgs::msg::BoolStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        if (cisstData.Type() == prmEventButton::PRESSED) {
            rosData.data = true;
        } else if (cisstData.Type() == prmEventButton::RELEASED) {
            rosData.data = false;
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmEventButton & cisstData, sensor_msgs::msg::Joy & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    rosData.axes.resize(0);
    rosData.buttons.resize(1);
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        if (cisstData.Type() == prmEventButton::PRESSED) {
            rosData.buttons[0] = 1;
        } else if (cisstData.Type() == prmEventButton::RELEASED) {
            rosData.buttons[0] = 0;
        } else if (cisstData.Type() == prmEventButton::CLICKED) {
            rosData.buttons[0] = 2;
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctDoubleMat & cisstData, std_msgs::msg::Float64MultiArray & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.layout.dim.resize(2);
    rosData.layout.dim[0].label  = "rows";
    rosData.layout.dim[0].size   = cisstData.rows();
    rosData.layout.dim[0].stride = 1;
    rosData.layout.dim[1].label  = "cols";
    rosData.layout.dim[1].size   = cisstData.cols();
    rosData.layout.dim[1].stride = cisstData.rows();
    rosData.layout.data_offset = 0;
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.data.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.data.begin());
    }
    return true;
}

bool mtsCISSTToROS(const vctDoubleVec & cisstData, std_msgs::msg::Float64MultiArray & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.layout.dim.resize(2);
    rosData.layout.dim[0].label  = "rows";
    rosData.layout.dim[0].size   = 1;
    rosData.layout.dim[0].stride = 1;
    rosData.layout.dim[1].label  = "cols";
    rosData.layout.dim[1].size   = cisstData.size();
    rosData.layout.dim[1].stride = 1;
    rosData.layout.data_offset = 0;
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.data.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.data.begin());
    }
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSTransform(cisstData.Position(), rosData);
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.header.frame_id = cisstData.ReferenceFrame();
        rosData.child_frame_id = cisstData.MovingFrame();
        mtsCISSTToROSTransform(cisstData.Position(), rosData.transform);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSPose(cisstData.Position(), rosData);
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::PoseStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.header.frame_id = cisstData.ReferenceFrame();
        mtsCISSTToROSPose(cisstData.Position(), rosData.pose);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmPositionCartesianArrayGet & cisstData, geometry_msgs::msg::PoseArray & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.header.frame_id = cisstData.ReferenceFrame();
        typedef std::vector<vctFrm3>::const_iterator IteratorType;
        const IteratorType end = cisstData.Positions().end();
        IteratorType iter;
        size_t index = 0;
        rosData.poses.resize(cisstData.Positions().size());
        for (iter = cisstData.Positions().begin();
             iter != end;
             ++iter, ++index) {
            mtsCISSTToROSPose(*iter, rosData.poses[index]);
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSPose(cisstData.Goal(), rosData);
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::PoseStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        mtsCISSTToROSPose(cisstData.Goal(), rosData.pose);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::msg::Pose & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        mtsCISSTToROSTransform(cisstData.Goal(), rosData.transform);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        mtsCISSTToROSTransform(cisstData, rosData.transform);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::msg::Transform & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::msg::Vector3 & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.x = cisstData[0];
    rosData.y = cisstData[1];
    rosData.z = cisstData[2];
    return true;
}

bool mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::msg::Quaternion & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    vctQuatRot3 quat(cisstData, VCT_NORMALIZE);
    rosData.x = quat.X();
    rosData.y = quat.Y();
    rosData.z = quat.Z();
    rosData.w = quat.W();
    return true;
}

bool mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::msg::QuaternionStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, node, debugInfo);
    vctQuatRot3 quat(cisstData, VCT_NORMALIZE);
    rosData.quaternion.x = quat.X();
    rosData.quaternion.y = quat.Y();
    rosData.quaternion.z = quat.Z();
    rosData.quaternion.w = quat.W();
    return true;
}

bool mtsCISSTToROS(const vct6 & cisstData, geometry_msgs::msg::Wrench & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSWrench(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vct6 & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, node, debugInfo);
    mtsCISSTToROSWrench(cisstData, rosData.wrench);
    return true;
}

bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::Wrench & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string & debugInfo)
{
    if (cisstData.size() != 6) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: wrench data size error, should be 6, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    mtsCISSTToROSWrench(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (cisstData.size() != 6) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: wrench data size error, should be 6, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo);
    mtsCISSTToROSWrench(cisstData, rosData.wrench);
    return true;
}

bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::Vector3Stamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (cisstData.size() != 3) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: vector data size error, should be 3, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.vector.x = cisstData.Element(0);
        rosData.vector.y = cisstData.Element(1);
        rosData.vector.z = cisstData.Element(2);
    }
    return true;
}

bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::msg::Twist & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    if (cisstData.Valid()) {
        rosData.linear.x = cisstData.VelocityLinear().X();
        rosData.linear.y = cisstData.VelocityLinear().Y();
        rosData.linear.z = cisstData.VelocityLinear().Z();
        rosData.angular.x = cisstData.VelocityAngular().X();
        rosData.angular.y = cisstData.VelocityAngular().Y();
        rosData.angular.z = cisstData.VelocityAngular().Z();
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::msg::TwistStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.header.frame_id = cisstData.MovingFrame();
        mtsCISSTToROS(cisstData, rosData.twist, node, debugInfo);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::msg::Wrench & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    if (cisstData.Valid()) {
        mtsCISSTToROSWrench(cisstData.Force(), rosData);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.header.frame_id = cisstData.MovingFrame();
        mtsCISSTToROS(cisstData, rosData.wrench, node, debugInfo);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceCartesianSet & cisstData, geometry_msgs::msg::Wrench & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    if (cisstData.Valid()) {
        mtsCISSTToROSWrench(cisstData.Force(), rosData);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceCartesianSet & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        mtsCISSTToROS(cisstData, rosData.wrench, node, debugInfo);
        return true;
    }
    return false;
}

// ---------------------------------------------
// sensor_msgs
// ---------------------------------------------
bool mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, node, debugInfo);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.position.begin());
    }
    return true;
}

bool mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.name.resize(0);
        rosData.velocity.resize(0);
        rosData.effort.resize(0);
        const size_t size = cisstData.Position().size();
        if (size != 0) {
            rosData.position.resize(size);
            std::copy(cisstData.Position().begin(), cisstData.Position().end(),
                      rosData.position.begin());
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmPositionJointSet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.name.resize(0);
        rosData.velocity.resize(0);
        rosData.effort.resize(0);
        const size_t size = cisstData.Goal().size();
        if (size != 0) {
            rosData.position.resize(size);
            std::copy(cisstData.Goal().begin(), cisstData.Goal().end(),
                      rosData.position.begin());
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.name.resize(0);
        rosData.position.resize(0);
        rosData.effort.resize(0);
        const size_t size = cisstData.Velocity().size();
        if (size != 0) {
            rosData.velocity.resize(size);
            std::copy(cisstData.Velocity().begin(), cisstData.Velocity().end(),
                      rosData.velocity.begin());
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceTorqueJointSet & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.name.resize(0);
        rosData.position.resize(0);
        rosData.velocity.resize(0);
        const size_t size = cisstData.ForceTorque().size();
        if (size != 0) {
            rosData.effort.resize(size);
            std::copy(cisstData.ForceTorque().begin(), cisstData.ForceTorque().end(),
                      rosData.effort.begin());
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::msg::JointState & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        { // names
            const size_t size = cisstData.Name().size();
            if (size != 0) {
                rosData.name.resize(size);
                std::copy(cisstData.Name().begin(), cisstData.Name().end(),
                          rosData.name.begin());
            }
        }
        { // positions
            const size_t size = cisstData.Position().size();
            if (size != 0) {
                rosData.position.resize(size);
                std::copy(cisstData.Position().begin(), cisstData.Position().end(),
                          rosData.position.begin());
            }
        }
        { // velocities
            const size_t size = cisstData.Velocity().size();
            if (size != 0) {
                rosData.velocity.resize(size);
                std::copy(cisstData.Velocity().begin(), cisstData.Velocity().end(),
                          rosData.velocity.begin());
            }
        }
        { // efforts
            const size_t size = cisstData.Effort().size();
            if (size != 0) {
                rosData.effort.resize(size);
                std::copy(cisstData.Effort().begin(), cisstData.Effort().end(),
                          rosData.effort.begin());
            }
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::msg::PointCloud & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, node, debugInfo);
    rosData.points.resize(cisstData.rows());
    for (size_t i = 0; i < cisstData.rows(); ++i) {
        rosData.points[i].x = cisstData.at(i, 0);
        rosData.points[i].y = cisstData.at(i, 1);
        rosData.points[i].z = cisstData.at(i, 2);
    }
    return true;
}

bool mtsCISSTToROS(const std::vector<vct3> & cisstData, sensor_msgs::msg::PointCloud & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, node, debugInfo);
    rosData.points.resize(cisstData.size());
    typedef std::vector<vct3>::const_iterator IteratorType;
    const IteratorType end = cisstData.end();
    IteratorType iter;
    size_t index = 0;
    for (iter = cisstData.begin();
         iter != end;
         ++iter, ++index) {
        rosData.points[index].x = iter->X();
        rosData.points[index].y = iter->Y();
        rosData.points[index].z = iter->Z();
    }
    return true;
}

bool mtsCISSTToROS(const prmInputData & cisstData, sensor_msgs::msg::Joy & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.axes.resize(cisstData.AnalogInputs().size());
        rosData.buttons.resize(cisstData.DigitalInputs().size());
        std::copy(cisstData.AnalogInputs().begin(), cisstData.AnalogInputs().end(),
                  rosData.axes.begin());
        std::copy(cisstData.DigitalInputs().begin(), cisstData.DigitalInputs().end(),
                  rosData.buttons.begin());
        return true;
    }
    return false;
}

// ---------------------------------------------
// diagnostic_msgs
// ---------------------------------------------
bool mtsCISSTToROS(const prmKeyValue & cisstData, diagnostic_msgs::msg::KeyValue & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.key = cisstData.Key;
    rosData.value = cisstData.Value;
    return true;
}


// ---------------------------------------------
// std_srvs
// ---------------------------------------------
bool mtsCISSTToROS(const bool & cisstData, std_srvs::srv::Trigger::Response & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.success = cisstData;
    rosData.message = "";
    return true;
}

bool mtsCISSTToROS(const std::string & cisstData, std_srvs::srv::Trigger::Response & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    rosData.success = true;
    rosData.message = cisstData;
    return true;
}

// ---------------------------------------------
// cisst_msgs
// ---------------------------------------------
bool mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::msg::DoubleVec & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        const size_t size = cisstData.Position().size();
        rosData.data.resize(size);
        for (size_t i = 0; i < size; ++i) {
            rosData.data[i] = cisstData.Position().Element(i);
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::msg::DoubleVec & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, node, debugInfo);
    const size_t size = cisstData.size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Element(i);
    }
    return true;
}

bool mtsCISSTToROS(const prmCartesianImpedanceGains & cisstData, cisst_msgs::msg::CartesianImpedanceGains & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        // vf pos/rot
        mtsCISSTToROS(cisstData.ForcePosition(),
                      rosData.force_position, node, debugInfo);
        mtsCISSTToROS(cisstData.ForceOrientation(),
                      rosData.force_orientation, node, debugInfo);
        mtsCISSTToROS(cisstData.TorqueOrientation(),
                      rosData.torque_orientation, node, debugInfo);

        // force gains
        mtsCISSTToROS(cisstData.PositionDeadbandPos(),
                      rosData.pos_deadband_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.PositionDeadbandNeg(),
                      rosData.pos_deadband_neg, node, debugInfo);
        mtsCISSTToROS(cisstData.PositionStiffnessPos(),
                      rosData.pos_stiff_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.PositionStiffnessNeg(),
                      rosData.pos_stiff_neg, node, debugInfo);
        mtsCISSTToROS(cisstData.PositionDampingPos(),
                      rosData.pos_damping_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.PositionDampingNeg(),
                      rosData.pos_damping_neg, node, debugInfo);
        mtsCISSTToROS(cisstData.ForceBiasPos(),
                      rosData.force_bias_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.ForceBiasNeg(),
                      rosData.force_bias_neg, node, debugInfo);

        // torque gains
        mtsCISSTToROS(cisstData.OrientationDeadbandPos(),
                      rosData.ori_deadband_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.OrientationDeadbandNeg(),
                      rosData.ori_deadband_neg, node, debugInfo);
        mtsCISSTToROS(cisstData.OrientationStiffnessPos(),
                      rosData.ori_stiff_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.OrientationStiffnessNeg(),
                      rosData.ori_stiff_neg, node, debugInfo);
        mtsCISSTToROS(cisstData.OrientationDampingPos(),
                      rosData.ori_damping_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.OrientationDampingNeg(),
                      rosData.ori_damping_neg, node, debugInfo);
        mtsCISSTToROS(cisstData.TorqueBiasPos(),
                      rosData.torque_bias_pos, node, debugInfo);
        mtsCISSTToROS(cisstData.TorqueBiasNeg(),
                      rosData.torque_bias_neg, node, debugInfo);

        return true;
    }
    return false;
}

bool mtsCISSTToROS(const mtsIntervalStatistics & cisstData, cisst_msgs::msg::IntervalStatistics & rosData,
                   std::shared_ptr<rclcpp::Node> node, const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, node, debugInfo)) {
        rosData.period_avg = cisstData.PeriodAvg();
        rosData.period_std_dev = cisstData.PeriodStdDev();
        rosData.period_min = cisstData.PeriodMin();
        rosData.period_max = cisstData.PeriodMax();
        rosData.compute_time_avg = cisstData.ComputeTimeAvg();
        rosData.compute_time_std_dev = cisstData.ComputeTimeStdDev();
        rosData.compute_time_min = cisstData.ComputeTimeMin();
        rosData.compute_time_max = cisstData.ComputeTimeMax();
        rosData.number_of_samples = cisstData.NumberOfSamples();
        rosData.number_of_overruns = cisstData.NumberOfOverruns();
        rosData.statistics_interval = cisstData.StatisticsInterval();
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctFrm4x4 & cisstData, cisst_msgs::srv::QueryForwardKinematics::Response & rosData,
                   std::shared_ptr<rclcpp::Node>, const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData.cp.pose);
    return true;
}
