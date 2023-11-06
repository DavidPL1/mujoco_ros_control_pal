///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#pragma once

#include <boost/shared_ptr.hpp>
#include <mujoco_ros_control_pal/internal/joint_state.h>

namespace control_toolbox
{
  class Pid;
}

namespace joint_limits_interface
{
  class PositionJointSaturationHandle;
  class PositionJointSoftLimitsHandle;
}

namespace mujoco_ros::control::pal::internal
{

class PositionJoint : public JointState
{
public:
  PositionJoint();

  void init(const mjModel               *m_ptr,
            mjData                      *d_ptr,
            const std::string           &resource_name,
            const ros::NodeHandle       &nh,
            const urdf::Model* const     urdf_model,
            hardware_interface::RobotHW *robot_hw) override;

  void write(const ros::Time     &time,
             const ros::Duration &period,
             bool                 in_estop) override;

  std::vector<std::string> getHardwareInterfaceTypes() override;

protected:
  using SoftLimitsHandle = joint_limits_interface::PositionJointSoftLimitsHandle;
  using SatLimitsHandle = joint_limits_interface::PositionJointSaturationHandle;
  using SoftLimitsHandlePtr = boost::shared_ptr<SoftLimitsHandle>;
  using SatLimitsHandlePtr = boost::shared_ptr<SatLimitsHandle>;
  using PidPtr = boost::shared_ptr<control_toolbox::Pid>;

  double hold_pos_cmd_;
  double pos_cmd_;
  double pos_min_;
  double pos_max_;
  double eff_max_;
  bool prev_in_estop_;

  PidPtr pid_;

  SatLimitsHandlePtr sat_limits_handle_;
  SoftLimitsHandlePtr soft_limits_handle_;
};

} // namespace mujoco_ros::control::pal::internal
