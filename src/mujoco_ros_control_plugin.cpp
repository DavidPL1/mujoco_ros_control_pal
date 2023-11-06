/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

// Boost
#include <boost/bind.hpp>

#include <mujoco_ros_control_pal/mujoco_ros_control_plugin.h>
#include <urdf/model.h>

#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <thread>

namespace mujoco_ros::control::pal
{

MujocoRosControlPlugin::~MujocoRosControlPlugin() {}

// Overloaded entry point
bool MujocoRosControlPlugin::load(const mjModel *m_ptr, mjData *d_ptr)
{
  ROS_INFO_STREAM("Loading mujoco_ros_control_pal plugin");

  // Error message if the model couldn't be found
  if (!m_ptr)
  {
    ROS_ERROR_STREAM_NAMED("loadThread", "parent model is NULL");
    return false;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return false;
  }

  ROS_ASSERT(rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  // Check rosparam sanity
  if (!rosparam_config_.hasMember("hardware")) {
      ROS_ERROR_NAMED("mujoco_ros_control", "MujocoRosControlPlugin expects a 'hardware' rosparam specifying at least "
                                              "the 'type' of hardware interface and a 'control_period'");
      return false;
  }

  if (rosparam_config_.hasMember("robot_namespace")) {
      robot_namespace_ = static_cast<std::string>(rosparam_config_["robot_namespace"]);
  }
  robot_nh_ = ros::NodeHandle(robot_namespace_);

  robot_description_ = rosparam_config_["hardware"].hasMember("robot_description") ?
                              (std::string)rosparam_config_["hardware"]["robot_description"] :
                              "robot_description";

  if (rosparam_config_["hardware"].hasMember("robotSimType")) {
      robot_hw_sim_type_str_ = (std::string)rosparam_config_["hardware"]["robotSimType"];
  } else {
    robot_hw_sim_type_str_ = "mujoco_ros_control_pal/DefaultRobotHWSim";
    ROS_DEBUG_STREAM("Using default plugin for RobotHWSim (none specified in config)\""<<robot_hw_sim_type_str_<<"\"");
  }

  if (rosparam_config_["hardware"].hasMember("control_period")) {
    control_period_ = ros::Duration(static_cast<double>(rosparam_config_["hardware"]["control_period"]));
	if (control_period_.toSec() < m_ptr->opt.timestep) {
	    ROS_WARN_STREAM("Desired controller update period (" << control_period_
		                << " s) is faster than the MuJoCo simulation timestep ("
		                << m_ptr->opt.timestep << " s).");
	} else if (control_period_.toSec() > m_ptr->opt.timestep) {
	    ROS_WARN_STREAM("Desired controller update period (" << control_period_
		                << " s) is slower than the MuJoCo simulation timestep ("
		                << m_ptr->opt.timestep << " s).");
    }
  } else {
    control_period_ = ros::Duration(m_ptr->opt.timestep);
    ROS_DEBUG_STREAM_NAMED("gazebo_ros_control","Control period not found in URDF/SDF, defaulting to MuJoCo step of "
      << control_period_);

  }

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;
  if (rosparam_config_["hardware"].hasMember("eStopTopic")) {
  	const std::string e_stop_topic = (std::string)rosparam_config_["hardware"]["eStopTopic"];
  	e_stop_sub_                    = robot_nh_.subscribe(e_stop_topic, 1, &MujocoRosControlPlugin::eStopCB, this);
  }

  ROS_INFO("Starting mujoco_ros_control_pal plugin in namespace: %s", robot_namespace_.c_str());

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR("Error parsing URDF in mujoco_ros_control_pal plugin, plugin not active.\n");
    return false;
  }

  // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
  try
  {
    robot_hw_sim_loader_.reset
      (new pluginlib::ClassLoader<mujoco_ros::control::pal::RobotHWSim>
        ("mujoco_ros_control_pal",
          "mujoco_ros::control::pal::RobotHWSim"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(!robot_hw_sim_->initSim(m_ptr, d_ptr, env_ptr_, robot_namespace_, robot_nh_, urdf_model_ptr, transmissions_))
    {
      ROS_FATAL("Could not initialize robot simulation interface");
      return false;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM("Loading controller_manager");
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), robot_nh_));
  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM("Failed to create robot simulation interface loader: "<<ex.what());
    return false;
  }

  ROS_INFO("Loaded gazebo_ros_control.");
  return true;
}

void MujocoRosControlPlugin::controlCallback(const mjModel * /*model*/, mjData *data)
{
  // Get the simulation time and period
  ros::Time sim_time_ros = ros::Time::now();
  ROS_WARN_STREAM_COND(sim_time_ros < ros::Time(data->time),
                        "ROS time not in sync with mjData! (" << sim_time_ros << " < " << ros::Time(data->time)
                                                                << ")");
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;
  bool reset_ctrlrs        = last_update_sim_time_ros_.isZero();

  robot_hw_sim_->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  if(sim_period >= control_period_ || (reset_ctrlrs && !sim_period.isZero())) {
    last_update_sim_time_ros_ = sim_time_ros;
    robot_hw_sim_->readSim(sim_time_ros, sim_period);

    if (e_stop_active_) {
      last_e_stop_active_ = true;
    } else if (last_e_stop_active_) {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
    }

    controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
  }

  if (!last_update_sim_time_ros_.isZero() && (sim_time_ros > last_write_sim_time_ros_)) {
    robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
    last_write_sim_time_ros_ = sim_time_ros;
  }
}

// Called on world reset
void MujocoRosControlPlugin::reset() {}

// Get the URDF XML from the parameter server
std::string MujocoRosControlPlugin::getURDF(const std::string &param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (robot_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE("mujoco_ros_control_pal plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      robot_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE("mujoco_ros_control_pal plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      robot_nh_.getParam(param_name, urdf_string);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  ROS_DEBUG_STREAM("Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool MujocoRosControlPlugin::parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

// Emergency stop callback
void MujocoRosControlPlugin::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
{
  e_stop_active_ = e_stop_active->data;
}
} // namespace mujoco_ros::control::pal

PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::pal::MujocoRosControlPlugin, mujoco_ros::MujocoPlugin)