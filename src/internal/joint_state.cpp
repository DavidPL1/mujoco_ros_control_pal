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

#include <cassert>
#include <stdexcept>

#include <angles/angles.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include<mujoco_ros_control_pal/internal/joint_state.h>

#include <mujoco_ros/util.h>

namespace mujoco_ros::control::pal::internal
{

JointState::JointState()
  : pos_(0.0),
    vel_(0.0),
    eff_(0.0)
{}

void JointState::init(const mjModel               *m_ptr,
                      mjData                      *d_ptr,
                      const std::string           &resource_name,
                      const ros::NodeHandle       &nh,
                      const urdf::Model* const     urdf_model,
                      hardware_interface::RobotHW *robot_hw)
{
  assert(m_ptr && robot_hw && urdf_model);

  // init resource name
  name_ = resource_name;
  m_ptr_ = m_ptr;
  d_ptr_ = d_ptr;

  // cache Gazebo joint
  mujoco_joint_id_ = mujoco_ros::util::jointName2id(const_cast<mjModel *>(m_ptr), resource_name);
  if (mujoco_joint_id_ < 0 ) { // id = -1 is not existing
    const std::string msg = "Joint '" + resource_name + "' not found in MuJoCo model.";
    throw std::runtime_error(msg);
  }

  // cache URDF joint
  urdf_joint_ = urdf_model->getJoint(resource_name);
  if (!urdf_joint_)
  {
    const std::string msg = "URDF model does not contain joint '" + resource_name + "'.";
    throw std::runtime_error(msg);
  }

  // ros_control hardware interface
  namespace hi  = hardware_interface;
  namespace hii = hi::internal;

  hi::JointStateInterface* js_iface = robot_hw->get<hi::JointStateInterface>();
  if (!js_iface)
  {
    const std::string msg = "Robot hardware abstraction does not have hardware interface '" +
                             hii::demangledTypeName<hi::JointStateInterface>() + "'.";
    throw std::runtime_error(msg);
  }

  // resource is already registered in hardware interface
  // NOTE: we don't perform this check and bail out earlier, as the initialization
  // of class members shared with child classes is still relevant. What we want to
  // actually avoid is adding the resource to the hardware interface multiple times,
  // as it prints undesired warnings
  if (hasResource(resource_name, *js_iface))
  {
    throw ExistingResourceException();
  }

  // register resource in ros_control hardware interface
  hi::JointStateHandle js_handle(resource_name, &pos_, &vel_, &eff_, &pos_, &eff_);
  js_iface->registerHandle(js_handle);
}

void JointState::read(const ros::Time&     /*time*/,
                      const ros::Duration& /*period*/,
                      bool                 /*in_estop*/)
{
  using angles::shortest_angular_distance;

  // Gazebo has an interesting API...
  if (urdf_joint_->type == urdf::Joint::PRISMATIC) {
    pos_ = d_ptr_->qpos[m_ptr_->jnt_qposadr[mujoco_joint_id_]];
  } else {
    pos_ += shortest_angular_distance(pos_, d_ptr_->qpos[m_ptr_->jnt_qposadr[mujoco_joint_id_]]);
  }
  vel_ = d_ptr_->qvel[m_ptr_->jnt_dofadr[mujoco_joint_id_]];
  eff_ = d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[mujoco_joint_id_]];
}

std::vector<std::string> JointState::getHardwareInterfaceTypes()
{
  namespace hi  = hardware_interface;
  namespace hii = hi::internal;

  std::vector<std::string> out;
  out.push_back(hii::demangledTypeName<hi::JointStateInterface>());
  return out;
}
} // namespace mujoco_ros::control::pal::internal
