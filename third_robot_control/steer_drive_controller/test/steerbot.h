///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
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

// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

template <unsigned int NUM_JOINTS = 2>
class Steerbot : public hardware_interface::RobotHW
{
public:
  Steerbot()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &Steerbot::start_callback, this))
  , stop_srv_(nh_.advertiseService("stop", &Steerbot::stop_callback, this))
  , ns_("steerbot_controller/")
  {
    // Intialize raw data
    std::fill_n(pos_, NUM_JOINTS, 0.0);
    std::fill_n(vel_, NUM_JOINTS, 0.0);
    std::fill_n(eff_, NUM_JOINTS, 0.0);
    std::fill_n(cmd_, NUM_JOINTS, 0.0);

    this->clean_up();
    this->get_joint_names(nh_);
    this->register_hardware_interfaces();

    /*
    // Connect and register the joint state and velocity interface
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << "wheel_" << i << "_joint";

      hardware_interface::JointStateHandle state_handle(os.str(), &pos_[i], &vel_[i], &eff_[i]);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface_.getHandle(os.str()), &cmd_[i]);
      jnt_vel_interface_.registerHandle(vel_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
    */
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read()
  {
    std::ostringstream os;
    for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    {
      os << cmd_[i] << ", ";
    }
    os << cmd_[NUM_JOINTS - 1];

    ROS_INFO_STREAM("Commands for joints: " << os.str());
  }

  void write()
  {
    if (running_)
    {
      for (unsigned int i = 0; i < NUM_JOINTS; ++i)
      {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        pos_[i] += vel_[i]*getPeriod().toSec(); // update position
        vel_[i] = cmd_[i]; // might add smoothing here later
      }
    }
    else
    {
      std::fill_n(pos_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
      std::fill_n(vel_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
    }
  }

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

private:
  
  void clean_up()
  {
    // wheel
    //-- wheel joint names
    wheel_jnt_name_.empty();
    virtual_rear_wheel_jnt_names_.clear();
    //-- actual rear wheel joint
    wheel_jnt_pos_ = 0;
    wheel_jnt_vel_ = 0;
    wheel_jnt_eff_ = 0;
    wheel_jnt_vel_cmd_ = 0;
    //-- virtual rear wheel joint
    virtual_rear_wheel_jnt_pos_.clear();
    virtual_rear_wheel_jnt_vel_.clear();
    virtual_rear_wheel_jnt_eff_.clear();
    virtual_rear_wheel_jnt_vel_cmd_.clear();
    //-- virtual front wheel joint
    virtual_front_wheel_jnt_pos_.clear();
    virtual_front_wheel_jnt_vel_.clear();
    virtual_front_wheel_jnt_eff_.clear();
    virtual_front_wheel_jnt_vel_cmd_.clear();

    // steer
    //-- steer joint names
    steer_jnt_name_.empty();
    virtual_steer_jnt_names_.clear();
    //-- front steer joint
    steer_jnt_pos_ = 0;
    steer_jnt_vel_ = 0;
    steer_jnt_eff_ = 0;
    steer_jnt_pos_cmd_ = 0;
    //-- virtual wheel joint
    virtual_steer_jnt_pos_.clear();
    virtual_steer_jnt_vel_.clear();
    virtual_steer_jnt_eff_.clear();
    virtual_steer_jnt_pos_cmd_.clear();
  }

  void get_joint_names(ros::NodeHandle &_nh)
  {
    this->get_wheel_joint_names(_nh);
    this->get_steer_joint_names(_nh);
  }

  void get_wheel_joint_names(ros::NodeHandle &_nh)
  {
    // wheel joint to get linear command
    _nh.getParam(ns_ + "rear_wheel", wheel_jnt_name_);

    // virtual wheel joint for gazebo control
    _nh.getParam(ns_ + "rear_wheels", virtual_rear_wheel_jnt_names_);
    int dof = virtual_rear_wheel_jnt_names_.size();
    virtual_rear_wheel_jnt_pos_.resize(dof);
    virtual_rear_wheel_jnt_vel_.resize(dof);
    virtual_rear_wheel_jnt_eff_.resize(dof);
    virtual_rear_wheel_jnt_vel_cmd_.resize(dof);

    _nh.getParam(ns_ + "front_wheels", virtual_front_wheel_jnt_names_);
    dof = virtual_front_wheel_jnt_names_.size();
    virtual_front_wheel_jnt_pos_.resize(dof);
    virtual_front_wheel_jnt_vel_.resize(dof);
    virtual_front_wheel_jnt_eff_.resize(dof);
    virtual_front_wheel_jnt_vel_cmd_.resize(dof);

  }

  void get_steer_joint_names(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", steer_jnt_name_);

    // virtual steer joint for gazebo control
    _nh.getParam(ns_ + "front_steers", virtual_steer_jnt_names_);

    const int dof = virtual_steer_jnt_names_.size();
    virtual_steer_jnt_pos_.resize(dof);
    virtual_steer_jnt_vel_.resize(dof);
    virtual_steer_jnt_eff_.resize(dof);
    virtual_steer_jnt_pos_cmd_.resize(dof);
  }

  void register_hardware_interfaces()
  {
    this->register_steer_interface();
    this->register_wheel_interface();

    // register mapped interface to ros_control
    registerInterface(&jnt_state_interface_);
    registerInterface(&wheel_jnt_vel_cmd_interface_);
    registerInterface(&steer_jnt_pos_cmd_interface_);
  }

  void register_wheel_interface()
  {
    // actual wheel joints
    this->register_interface_handles(
          jnt_state_interface_, wheel_jnt_vel_cmd_interface_,
          wheel_jnt_name_, wheel_jnt_pos_, wheel_jnt_vel_, wheel_jnt_eff_, wheel_jnt_vel_cmd_);

    // virtual rear wheel joints
    for (int i = 0; i < virtual_rear_wheel_jnt_names_.size(); i++)
    {
      this->register_interface_handles(
            jnt_state_interface_, wheel_jnt_vel_cmd_interface_,
            virtual_rear_wheel_jnt_names_[i], virtual_rear_wheel_jnt_pos_[i], virtual_rear_wheel_jnt_vel_[i], virtual_rear_wheel_jnt_eff_[i], virtual_rear_wheel_jnt_vel_cmd_[i]);
    }
    // virtual front wheel joints
    for (int i = 0; i < virtual_front_wheel_jnt_names_.size(); i++)
    {
      this->register_interface_handles(
            jnt_state_interface_, wheel_jnt_vel_cmd_interface_,
            virtual_front_wheel_jnt_names_[i], virtual_front_wheel_jnt_pos_[i], virtual_front_wheel_jnt_vel_[i], virtual_front_wheel_jnt_eff_[i], virtual_front_wheel_jnt_vel_cmd_[i]);
    }
  }

  void register_steer_interface()
  {
    // actual steer joints
    this->register_interface_handles(
          jnt_state_interface_, steer_jnt_pos_cmd_interface_,
          steer_jnt_name_, steer_jnt_pos_, steer_jnt_vel_, steer_jnt_eff_, steer_jnt_pos_cmd_);

    // virtual steer joints
    for (int i = 0; i < virtual_steer_jnt_names_.size(); i++)
    {
      this->register_interface_handles(
            jnt_state_interface_, steer_jnt_pos_cmd_interface_,
            virtual_steer_jnt_names_[i], virtual_steer_jnt_pos_[i], virtual_steer_jnt_vel_[i], virtual_steer_jnt_eff_[i], virtual_steer_jnt_pos_cmd_[i]);
    }
  }

  void register_interface_handles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          hardware_interface::JointCommandInterface& _jnt_cmd_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff,  double& _jnt_cmd)
  {
    // register joint (both JointState and CommandJoint)
    this->register_joint_state_interface_handle(_jnt_state_interface, _jnt_name,
                                            _jnt_pos, _jnt_vel, _jnt_eff);
    this->register_command_joint_interface_handle(_jnt_state_interface, _jnt_cmd_interface,
                                              _jnt_name, _jnt_cmd);
  }

  void register_joint_state_interface_handle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff)
  {
    hardware_interface::JointStateHandle state_handle(_jnt_name,
                                                      &_jnt_pos,
                                                      &_jnt_vel,
                                                      &_jnt_eff);
    _jnt_state_interface.registerHandle(state_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the JointStateInterface");
  }

  void register_command_joint_interface_handle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      hardware_interface::JointCommandInterface& _jnt_cmd_interface,
      const std::string _jnt_name, double& _jnt_cmd)
  {
    // joint command
    hardware_interface::JointHandle _handle(_jnt_state_interface.getHandle(_jnt_name),
                                            &_jnt_cmd);
    _jnt_cmd_interface.registerHandle(_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the CommandJointInterface");
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[NUM_JOINTS];
  double pos_[NUM_JOINTS];
  double vel_[NUM_JOINTS];
  double eff_[NUM_JOINTS];


  // rear wheel
  //-- actual joint(single actuator)
  //---- joint name
  std::string wheel_jnt_name_;
  //---- joint interface parameters
  double wheel_jnt_pos_;
  double wheel_jnt_vel_;
  double wheel_jnt_eff_;
  //---- joint interface command
  double wheel_jnt_vel_cmd_;
  //---- Hardware interface: joint
  hardware_interface::VelocityJointInterface wheel_jnt_vel_cmd_interface_;
  //hardware_interface::JointStateInterface wheel_jnt_state_interface_;
  //
  //-- virtual joints(two rear wheels)
  //---- joint name
  std::vector<std::string> virtual_rear_wheel_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_rear_wheel_jnt_pos_;
  std::vector<double> virtual_rear_wheel_jnt_vel_;
  std::vector<double> virtual_rear_wheel_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_rear_wheel_jnt_vel_cmd_;
  //-- virtual joints(two front wheels)
  //---- joint name
  std::vector<std::string> virtual_front_wheel_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_front_wheel_jnt_pos_;
  std::vector<double> virtual_front_wheel_jnt_vel_;
  std::vector<double> virtual_front_wheel_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_front_wheel_jnt_vel_cmd_;

  // front steer
  //-- actual joint(single actuator)
  //---- joint name
  std::string steer_jnt_name_;
  //---- joint interface parameters
  double steer_jnt_pos_;
  double steer_jnt_vel_;
  double steer_jnt_eff_;
  //---- joint interface command
  double steer_jnt_pos_cmd_;
  //---- Hardware interface: joint
  hardware_interface::PositionJointInterface steer_jnt_pos_cmd_interface_;
  //hardware_interface::JointStateInterface steer_jnt_state_interface_;
  //
  //-- virtual joints(two steers)
  //---- joint name
  std::vector<std::string> virtual_steer_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_steer_jnt_pos_;
  std::vector<double> virtual_steer_jnt_vel_;
  std::vector<double> virtual_steer_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_steer_jnt_pos_cmd_;

  std::string ns_;
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
