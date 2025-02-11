// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <hydrus/hydrus_robot_model.h>

class HydrusTiltedRobotModel : public HydrusRobotModel {
public:
  HydrusTiltedRobotModel(bool init_with_rosparam = true,
                         bool verbose = false,
                         double fc_t_min_thre = 0,
                         double epsilon = 10);
  virtual ~HydrusTiltedRobotModel() = default;

  virtual void calcStaticThrust() override;

  Eigen::MatrixXd getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset = KDL::Vector::Zero()) override;

  void calcCoGMomentumJacobian() override;
  void updateJacobians(const KDL::JntArray& joint_positions, bool update_model = true) override;

  template <class T> T getGimbalProcessedJoint();

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  void gimbalsCtrlCallback(const sensor_msgs::JointStateConstPtr& gimbals_ctrl_msg);

  ros::Subscriber gimbals_sub_;

  Eigen::MatrixXd gimbal_jacobian_;

  KDL::JntArray gimbal_processed_joint_;
  std::mutex gimbal_processed_joint_mutex_;
  std::vector<KDL::Rotation> gimbal_rotation_from_link_;

};

template<> inline KDL::JntArray HydrusTiltedRobotModel::getGimbalProcessedJoint()
{
  std::lock_guard<std::mutex> lock(gimbal_processed_joint_mutex_);
  return gimbal_processed_joint_;
}

template<> inline sensor_msgs::JointState HydrusTiltedRobotModel::getGimbalProcessedJoint()
{
  return kdlJointToMsg(getGimbalProcessedJoint<KDL::JntArray>());
}
