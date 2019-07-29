/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_PLUGINS_EDITEDACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_EDITEDACTORPLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "gazebo_plugins/gazebo_ros_utils.h"
#include <ros/callback_queue.h>
#include <math.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>
#include <mutex> 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include "ros/subscribe_options.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


namespace gazebo
{
  class GAZEBO_VISIBLE EditedActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: EditedActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    private: void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    private: ignition::math::Vector3d velocity;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    // /// \brief short term target
    // private: ignition::math::Vector3d short_target;

    /// \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;

    private: double x_ =3;
    double y_ = 1;
    // std::string command_topic_;
    // GazeboRosPtr gazebo_ros_;
    void cmdvel_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    void QueueThread();
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    ignition::math::Pose3d initial_pose;
    ros::ServiceClient sc;
    // ros::ServiceClient map_gen;
    nav_msgs::GetPlan theplan;
    std_srvs::Empty empty_nodehandler;
    std::string actor_plan;
    ignition::math::Pose3d init_pose;
    ignition::math::Pose3d default_pose;
    std::mutex target_changed;
    struct path_coordinates{
      double x;
      double y;
    };
    std::vector<path_coordinates> global_path; 
    double updateRate;
    // //publish for animated_box_odom
    // std::string published_odom;
    // ros::Publisher map_pub_;
    // ros::NodeHandle nh_;
    // nav_msgs::Odometry odom_actor;
  };
}
#endif
