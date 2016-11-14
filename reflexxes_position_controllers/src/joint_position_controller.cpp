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

#include "joint_position_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sstream>

namespace reflexxes_position_controllers {

JointPositionController::JointPositionController()
    : loop_count_(0),
      decimation_(10),
      sampling_resolution_(0.001),
      new_reference_(false),
      must_recompute_trajectory_(false)
{}

JointPositionController::~JointPositionController() {
    trajectory_command_sub_.shutdown();
}


template<class T>
std::ostream &operator<< (std::ostream &stream, const RMLVector<T> &rml_vec) {
    stream << "[ ";

    for (int i = 0; i < rml_vec.VectorDimension; i++) {
        stream << (rml_vec.VecData[i]) << ", ";
    }

    stream << "]";
    return stream;
}

void JointPositionController::rml_debug(const ros::console::levels::Level level) {
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT NumberOfDOFs: " << rml_in_->NumberOfDOFs);
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT MinimumSynchronizationTime: " << rml_in_->MinimumSynchronizationTime);
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT SelectionVector: " << (*rml_in_->SelectionVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT CurrentPositionVector: " << (*rml_in_->CurrentPositionVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT CurrentVelocityVector: " << (*rml_in_->CurrentVelocityVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT CurrentAccelerationVector: " << (*rml_in_->CurrentAccelerationVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT MaxAccelerationVector: " << (*rml_in_->MaxAccelerationVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT MaxJerkVector: " << (*rml_in_->MaxJerkVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT TargetVelocityVector: " << (*rml_in_->TargetVelocityVector));

    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT MaxVelocityVector: " << (*rml_in_->MaxVelocityVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT TargetPositionVector: " << (*rml_in_->TargetPositionVector));
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, "RML INPUT AlternativeTargetVelocityVector: " 
        << (*rml_in_->AlternativeTargetVelocityVector));
}


bool JointPositionController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {
    // Store nodehandle
    nh_ = n;

    // Get joint names
    XmlRpc::XmlRpcValue xml_array;

    if (!nh_.getParam("joint_names", xml_array)) {
        ROS_ERROR("No 'joint_names' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
        return false;
    }

    // Make sure it's an array type
    if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("The 'joint_names' parameter is not an array (namespace '%s')", nh_.getNamespace().c_str());
        return false;
    }

    // Get number of joints
    n_joints_ = xml_array.size();

    ROS_INFO_STREAM("Initializing JointPositionController with " << n_joints_ << " joints.");

    // Get trajectory sampling resolution
    if (!nh_.hasParam("sampling_resolution")) {
        ROS_INFO("No sampling_resolution specified (namespace: %s), using default.", nh_.getNamespace().c_str());
    }

    nh_.param("sampling_resolution", sampling_resolution_, 0.001);
    
    // Get behavior after reaching point
    if (!nh_.hasParam("recompute_trajectory")) {
        ROS_INFO("No behavior after reaching point specified (namespace: %s), using default (keep trajectory).", nh_.getNamespace().c_str());
    }

    nh_.param("recompute_trajectory", recompute_trajectory_, false);

    // Create trajectory generator
    rml_.reset(new ReflexxesAPI(n_joints_, sampling_resolution_));
    rml_in_.reset(new RMLPositionInputParameters(n_joints_));
    rml_out_.reset(new RMLPositionOutputParameters(n_joints_));

    // Get urdf
    urdf::Model urdf;
    std::string urdf_str;
    ros::NodeHandle nh;
    nh.getParam("/robot_description", urdf_str);

    if (!urdf.initString(urdf_str)) {
        ROS_ERROR("Failed to parse urdf from '/robot_description' parameter (namespace: %s)", nh.getNamespace().c_str());
        return false;
    }

    // Get individual joint properties from urdf and parameter server
    joint_names_.resize(n_joints_);
    joints_.resize(n_joints_);
    urdf_joints_.resize(n_joints_);
    position_tolerances_.resize(n_joints_);
    max_velocities_.resize(n_joints_);
    max_accelerations_.resize(n_joints_);
    max_jerks_.resize(n_joints_);
    commanded_positions_.resize(n_joints_);

    for (int i = 0; i < n_joints_; i++) {
        // Get joint name
        if (xml_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("The 'joint_names' parameter contains a non-string element (namespace '%s')", nh_.getNamespace().c_str());
            return false;
        }

        joint_names_[i] = static_cast<std::string>(xml_array[i]);

        // Get the joint-namespace nodehandle
        {
            ros::NodeHandle joint_nh(nh_, "joints/" + joint_names_[i]);
            ROS_INFO("Loading joint information for joint '%s' (namespace: %s)", 
                     joint_names_[i].c_str(), joint_nh.getNamespace().c_str());

            // Get position tolerance
            if (!joint_nh.hasParam("position_tolerance")) {
                ROS_INFO("No position_tolerance specified (namespace: %s), using default.",
                         joint_nh.getNamespace().c_str());
            }

            joint_nh.param("position_tolerance", position_tolerances_[i], 0.1);
            
            // Get maximum velocity
            if (!joint_nh.hasParam("max_velocity")) {
                ROS_INFO("No max_velocity specified (namespace: %s), using default from URDF.",
                         joint_nh.getNamespace().c_str());
            }

            joint_nh.param("max_velocity", max_velocities_[i], 0.0);

            // Get maximum acceleration
            if (!joint_nh.hasParam("max_acceleration")) {
                ROS_INFO("No max_acceleration specified (namespace: %s), using default.",
                         joint_nh.getNamespace().c_str());
            }

            joint_nh.param("max_acceleration", max_accelerations_[i], 1.0);

            // Get maximum jerk
            if (!joint_nh.hasParam("max_jerk")) {
                ROS_INFO("No max_jerk specified (namespace: %s), using default.",
                         joint_nh.getNamespace().c_str());
            }

            joint_nh.param("max_jerk", max_jerks_[i], 1000.0);
        }

        // Get ros_control joint handle
        joints_[i] = robot->getHandle(joint_names_[i]);

        // Get urdf joint
        urdf_joints_[i] = urdf.getJoint(joint_names_[i]);

        if (!urdf_joints_[i]) {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return false;
        }

        // Get RML parameters from URDF
        rml_in_->MaxVelocityVector->VecData[i] = max_velocities_[i] == 0 ? urdf_joints_[i]->limits->velocity : max_velocities_[i];
        rml_in_->MaxAccelerationVector->VecData[i] = max_accelerations_[i];
        rml_in_->MaxJerkVector->VecData[i] = max_jerks_[i];
    }

    for (int i = 0; i < n_joints_; i++) {
        rml_in_->SelectionVector->VecData[i] = true;
    }


    if (rml_in_->CheckForValidity()) {
        ROS_INFO_STREAM("RML INPUT Configuration Valid.");
        this->rml_debug(ros::console::levels::Debug);
    } else {
        ROS_ERROR_STREAM("RML INPUT Configuration Invalid!");
        this->rml_debug(ros::console::levels::Warn);
        return false;
    }

    // Create state publisher
    // TODO: create state publisher
    //controller_state_publisher_.reset(
    //new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

    // Create command subscriber
    trajectory_command_sub_ = nh_.subscribe<trajectory_msgs::JointTrajectoryPoint>(
                                  "joint_position_command", 1, &JointPositionController::trajectoryCommandCB, this);

    return true;
}



void JointPositionController::starting(const ros::Time &time) {
    // Define an initial command point from the current position
    trajectory_msgs::JointTrajectoryPoint initial_point;

    for (int i = 0; i < n_joints_; i++) {
        initial_point.positions.push_back(joints_[i].getPosition());
        initial_point.velocities.push_back(joints_[i].getVelocity());
        initial_point.accelerations.push_back(0.0);
    }

    initial_point.time_from_start = ros::Duration(1.0);
    trajectory_command_buffer_.initRT(initial_point);

    // Reset commands
    for (int i = 0; i < n_joints_; i++) {
        commanded_positions_[i] = joints_[i].getPosition();
    }

    // Set new reference flag for initial command point
    new_reference_ = true;
}

void JointPositionController::update(const ros::Time &time, const ros::Duration &period) {
    // Read the latest commanded trajectory message
    const trajectory_msgs::JointTrajectoryPoint &commanded_trajectory = *(trajectory_command_buffer_.readFromRT());

    // Check for a new reference
    if (new_reference_) {

        // Reset new reference flag
        new_reference_ = false;
        // Set flag to recompute trajectory
        must_recompute_trajectory_ = true;

        ROS_DEBUG("Received new reference.");
    }

    // Initialize RML result
    int rml_result = 0;


    // Compute RML traj after the start time and if there are still points in the queue
    if (must_recompute_trajectory_) {
        // Compute the trajectory
        ROS_DEBUG("RML Recomputing trajectory...");

        // Update RML input parameters
        for (int i = 0; i < n_joints_; i++) {
            rml_in_->CurrentPositionVector->VecData[i] = joints_[i].getPosition();
            rml_in_->CurrentVelocityVector->VecData[i] = joints_[i].getVelocity();
            rml_in_->CurrentAccelerationVector->VecData[i] = 0.0;

            rml_in_->TargetPositionVector->VecData[i] = commanded_trajectory.positions[i];
            rml_in_->TargetVelocityVector->VecData[i] = commanded_trajectory.velocities[i];

            rml_in_->SelectionVector->VecData[i] = true;
        }
        
        
        ROS_DEBUG_STREAM("Current position: " << std::endl << *(rml_in_->CurrentPositionVector));
        ROS_DEBUG_STREAM("Target position: " << std::endl << *(rml_in_->TargetPositionVector));

        // Store the traj start time
        traj_start_time_ = time;

        // Set desired execution time for this trajectory (definitely > 0)
//         rml_in_->SetMinimumSynchronizationTime(0);

//         ROS_DEBUG_STREAM("RML IN: time: " << rml_in_->GetMinimumSynchronizationTime());

        // Specify behavior after reaching point
        rml_flags_.BehaviorAfterFinalStateOfMotionIsReached = recompute_trajectory_ ? RMLPositionFlags::RECOMPUTE_TRAJECTORY : RMLPositionFlags::KEEP_TARGET_VELOCITY;
        rml_flags_.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

        // Compute trajectory
        rml_result = rml_->RMLPosition(*rml_in_.get(),
                                       rml_out_.get(),
                                       rml_flags_);

        // Disable recompute flag
        must_recompute_trajectory_ = false;
    } else {
        // Sample the already computed trajectory
        rml_result = rml_->RMLPositionAtAGivenSampleTime(
                         (time - traj_start_time_).toSec(),
                         rml_out_.get());
    }

    // Determine if any of the joint tolerances have been violated
    for (int i = 0; i < n_joints_; i++) {
        double tracking_error = std::abs(rml_out_->NewPositionVector->VecData[i] - joints_[i].getPosition());

        if (tracking_error > position_tolerances_[i]) {
            must_recompute_trajectory_ = true;
            ROS_WARN_STREAM("Tracking for joint " << i << " outside of tolerance! (" << tracking_error 
                << " > " << position_tolerances_[i] << ")");
        }
    }

    // Compute command
    for (int i = 0; i < n_joints_; i++) {
        // Convenience variables
        double pos_actual = joints_[i].getPosition();

        double pos_target = rml_out_->NewPositionVector->VecData[i];

        commanded_positions_[i] = pos_target;
    }

    // Only set a different position command if the
    switch (rml_result) {
    case ReflexxesAPI::RML_WORKING:
        // S'all good.
        break;

    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        ROS_DEBUG("final state reached");
        must_recompute_trajectory_ = true;
        break;

    default:
        if (loop_count_ % decimation_ == 0) {
            ROS_ERROR("Reflexxes error code: %d. Setting position commands to measured position.", rml_result);
        }

        for (int i = 0; i < n_joints_; i++)
            commanded_positions_[i] = joints_[i].getPosition();
        
        break;
    };

    // Set the lower-level commands
    ROS_DEBUG("setting command");
    for (int i = 0; i < n_joints_; i++) {
        joints_[i].setCommand(commanded_positions_[i]);
    }

    // Publish state
    if (loop_count_ % decimation_ == 0) {
        /*
         *      boost::scoped_ptr<realtime_tools::RealtimePublisher<controllers_msgs::JointControllerState> >
         *        &state_pub = controller_state_publisher_;
         *
         *      for(int i=0; i<n_joints_; i++) {
         *        if(state_pub && state_pub->trylock()) {
         *          state_pub->msg_.header.stamp = time;
         *          state_pub->msg_.set_point = pos_target;
         *          state_pub->msg_.process_value = pos_actual;
         *          state_pub->msg_.process_value_dot = vel_actual;
         *          state_pub->msg_.error = pos_error;
         *          state_pub->msg_.time_step = period.toSec();
         *          state_pub->msg_.command = commanded_effort;
         *
         *          double dummy;
         *          pids_[i]->getGains(
         *              state_pub->msg_.p,
         *              state_pub->msg_.i,
         *              state_pub->msg_.d,
         *              state_pub->msg_.i_clamp,
         *              dummy);
         *          state_pub->unlockAndPublish();
        }
        }
        */
    }

    // Increment the loop count
    loop_count_++;
}

void JointPositionController::trajectoryCommandCB(
    const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {
    this->setTrajectoryCommand(msg);
}

void JointPositionController::setTrajectoryCommand(
    const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {
    ROS_INFO("Received new command");
    // the writeFromNonRT can be used in RT, if you have the guarantee that
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    trajectory_command_buffer_.writeFromNonRT(*msg);
    new_reference_ = true;
}


} // namespace

PLUGINLIB_EXPORT_CLASS(
    reflexxes_position_controllers::JointPositionController,
    controller_interface::ControllerBase)
