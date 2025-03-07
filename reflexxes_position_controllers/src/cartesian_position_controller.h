/*********************************************************************
 * Software License Agreement (LGPL License)
 *
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *
 *********************************************************************/

#ifndef POSITION_CONTROLLERS_JOINT_POSITION_CONTROLLER_H
#define POSITION_CONTROLLERS_JOINT_POSITION_CONTROLLER_H

/**
  @class reflexxes_position_controllers::CartesianPositionController
  @brief Cartesian Position Controller

  This class controls cartesian position.

  @section ROS ROS interface

  @param type Must be "reflexxes_position_controllers::CartesianPositionController"
  @param joint Name of the joint to control.

  Subscribes to:

  - @b command (std_msgs::PoseStamped) : The cartesian position to achieve.

Publishes:

- @b state (control_msgs::JointControllerState) :
Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

#include <geometry_msgs/PoseStamped.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <trac_ik/trac_ik.hpp>

namespace reflexxes_position_controllers {

class CartesianPositionController: public controller_interface::Controller<hardware_interface::PositionJointInterface> {

public:
    CartesianPositionController();
    ~CartesianPositionController();

public:
    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
    void starting(const ros::Time &time);
    void stopping(const ros::Time &time) { };
    void update(const ros::Time &time, const ros::Duration &period);

public:
    /**< Last commanded position. */
    realtime_tools::RealtimeBuffer<geometry_msgs::PoseStamped> trajectory_command_buffer_;

    size_t n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<double> position_tolerances_;
    std::vector<double> max_accelerations_;
    std::vector<double> max_jerks_;
    std::vector<double> commanded_positions_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr > urdf_joints_;

private:
    ros::NodeHandle nh_;
    int loop_count_;
    int decimation_;

    void rml_debug(const ros::console::levels::Level level);
    
    //! Kinematic solvers
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    KDL::JntArray current_joint_position;
    KDL::Frame target_cart_position;
    KDL::JntArray target_joint_position;
    std::string root_name;
    std::string tip_name;
    
    //! Acceleration computation
    KDL::JntArray previous_joint_velocity;
    KDL::JntArray current_joint_acceleration;

    //! Trajectory Generator
    boost::shared_ptr<ReflexxesAPI> rml_;
    boost::shared_ptr<RMLPositionInputParameters> rml_in_;
    boost::shared_ptr<RMLPositionOutputParameters> rml_out_;
    RMLPositionFlags rml_flags_;
    ros::Time traj_start_time_;

    //! Trajectory parameters
    double sampling_resolution_;
    bool new_reference_;
    bool recompute_trajectory_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_ ;

    // Command subscriber
    ros::Subscriber trajectory_command_sub_;
    void trajectoryCommandCB(const geometry_msgs::PoseStampedConstPtr &msg);
    void setTrajectoryCommand(const geometry_msgs::PoseStampedConstPtr &msg);
};

} // namespace

#endif
