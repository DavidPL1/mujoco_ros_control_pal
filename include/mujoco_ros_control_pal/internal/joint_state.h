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

#include <algorithm>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <mujoco_ros_control_pal/internal/read_write_resource.h>

#include <urdf/model.h>

namespace hardware_interface
{
  class RobotHW;
}

namespace mujoco_ros::control::pal::internal
{

// TODO: Create an abstract base class?, separate read from write?
class JointState : public ReadWriteResource
{
public:
  JointState();

  // TODO doc requisite: RobotHW must contain interface
  void init(const mjModel               *m_ptr,
            mjData                      *d_ptr,
            const std::string           &resource_name,
            const ros::NodeHandle       &nh,
            const urdf::Model* const     urdf_model,
            hardware_interface::RobotHW *robot_hw) override;

  void read(const ros::Time     &time,
            const ros::Duration &period,
            bool                 in_estop) override;

  std::vector<std::string> getHardwareInterfaceTypes() override;

  std::string getName() const override {return name_;}

protected:
  std::string name_;
  double pos_;
  double vel_;
  double eff_;

  urdf::JointConstSharedPtr urdf_joint_;

  int mujoco_joint_id_;
  const mjModel *m_ptr_;
  mjData *d_ptr_;
};

// Code copied almost verbatim from transmission_interface::JointStateInterfaceProvider
// TODO: Refactor to avoid duplication
template <class Interface>
bool hasResource(const std::string& name, const Interface& iface)
{
  using hardware_interface::internal::demangledTypeName;

  // Do nothing if resource already exists on the interface
  const std::vector<std::string>& existing_res = iface.getNames();
  return existing_res.end() != std::find(existing_res.begin(), existing_res.end(), name);
}

template <class T>
T clamp(const T& val, const T& min_val, const T& max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

} // namespace mujoco_ros::control::pal::internal
