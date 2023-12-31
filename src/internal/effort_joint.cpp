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

#include <algorithm>
#include <cassert>
#include <limits>
#include <stdexcept>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/node_handle.h>

#include<mujoco_ros_control_pal/internal/effort_joint.h>

namespace mujoco_ros::control::pal::internal
{

EffortJoint::EffortJoint()
  : JointState(),
    eff_cmd_(0.0)
{}

// NOTE: Untested class
void EffortJoint::init(const mjModel               *m_ptr,
                       mjData                      *d_ptr,
                       const std::string           &resource_name,
                       const ros::NodeHandle       &nh,
                       const urdf::Model* const     urdf_model,
                       hardware_interface::RobotHW *robot_hw)
{
  // initialize joint state interface
  try
  {
    JointState::init(m_ptr,
                     d_ptr,
                     resource_name,
                     nh,
                     urdf_model,
                     robot_hw);
  }
  catch (const internal::ExistingResourceException&) {} // resource already added, no problem

  // ros_control hardware interface
  namespace hi  = hardware_interface;
  namespace hii = hi::internal;

  hi::JointStateInterface* js_iface = robot_hw->get<hi::JointStateInterface>();
  assert(js_iface);                                                 // should be valid
  hi::JointStateHandle js_handle = js_iface->getHandle(resource_name); // should not throw

  hi::EffortJointInterface* eff_iface = robot_hw->get<hi::EffortJointInterface>();
  if (!eff_iface)
  {
    const std::string msg = "Robot hardware abstraction does not have hardware interface '" +
                             hii::demangledTypeName<hi::EffortJointInterface>() + "'.";
    throw std::runtime_error(msg);
  }

  // resource is already registered in hardware interface
  if (hasResource(resource_name, *eff_iface))
  {
    throw ExistingResourceException();
  }
  // register resource in ros_control hardware interface
  hi::JointHandle eff_handle(js_handle, &eff_cmd_);
  eff_iface->registerHandle(eff_handle);

  // get joint limits, if specified
  namespace jli = joint_limits_interface;
  jli::JointLimits limits;
  jli::SoftJointLimits soft_limits;

  const bool has_joint_limits = jli::getJointLimits(urdf_joint_, limits) ||
                                jli::getJointLimits(resource_name, nh, limits);
  const bool has_soft_joint_limits = jli::getSoftJointLimits(urdf_joint_, soft_limits);

  // TODO: Move to method?
  // joint limit enforcing
  // limits enforcement can be ignored for this joint by setting a ROS parameter
  bool ignore_limits = true;
  nh.getParam("joint_limits/ignore_joints/" + resource_name, ignore_limits);
  if (!ignore_limits && has_joint_limits)
  {
    if (has_soft_joint_limits)
    {
      soft_limits_handle_.reset(new SoftLimitsHandle(eff_handle, limits, soft_limits));
      ROS_DEBUG_STREAM("Soft joint limits will be enforced for joint '" << resource_name << "' when using the '" <<
                       hii::demangledTypeName<hi::EffortJointInterface>() << "'hardware interface.");
    }
    else
    {
      sat_limits_handle_.reset(new SatLimitsHandle(eff_handle, limits));
      ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << resource_name << "' when using the '" <<
                       hii::demangledTypeName<hi::EffortJointInterface>() << "'hardware interface.");
    }
  }
  else
  {
    ROS_DEBUG_STREAM("No joint limits will be enforced for joint '" << resource_name << "' when using the '" <<
                     hii::demangledTypeName<hi::EffortJointInterface>() << "'hardware interface.");
  }
}

void EffortJoint::write(const ros::Time&     /*time*/,
                        const ros::Duration &period,
                        bool                 in_estop)
{
  // NOTE: Currently effort joint limits enforcing is closed-loop, so no reset is required, as for the position joint
  // case

  // enforce joint limits
  if (soft_limits_handle_)
  {
    soft_limits_handle_->enforceLimits(period);
  }
  else if (sat_limits_handle_)
  {
    sat_limits_handle_->enforceLimits(period);
  }

  // stop joint if e-stop is active
  // NOTE: This policy should not be baked-in, but should be an orthogonal design choice instead
  const double eff_cmd = in_estop ? 0.0 : eff_cmd_;

  // write command
  d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[mujoco_joint_id_]] = eff_cmd;
}

std::vector<std::string> EffortJoint::getHardwareInterfaceTypes()
{
  namespace hi  = hardware_interface;
  namespace hii = hi::internal;

  std::vector<std::string> out = JointState::getHardwareInterfaceTypes();
  out.push_back(hii::demangledTypeName<hi::EffortJointInterface>());
  return out;
}

} // namespace mujoco_ros::control::pal::internal
