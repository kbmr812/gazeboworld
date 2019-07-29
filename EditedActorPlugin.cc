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

#include <functional>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/physics/physics.hh"
//#include "plugins/ActorPlugin.hh"
#include "EditedActorPlugin.hh"
#include <sdf/sdf.hh>
#include <mutex> 



using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(EditedActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
EditedActorPlugin::EditedActorPlugin()
{
}

/////////////////////////////////////////////////
void EditedActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // gazebo_ros_=GazeboRosPtr( new GazeboRos(_model,_sdf,"editActor"));
  // gazebo_ros_->isInitialized();
  // gazebo_ros_->getParameter<std::string> (command_topic_,"commandTopic","cmd_vel");

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  //this->actor->Update();
  this->world = this->actor->GetWorld();
  // init_pose(3.0,1.0,1.0138,1.5707,0,1.5707);
  

  //Onupdate loop running in background
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&EditedActorPlugin::OnUpdate, this, std::placeholders::_1)));

  //Read the initial pose of Actor
  if(_sdf->HasElement("pose"))
    this->init_pose = _sdf->Get<ignition::math::Pose3d>("pose");
  else
    this->init_pose.Set(0,0,0,1.5707,0,0);
  
  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  //set actor world pose
  this->actor->SetWorldPose(init_pose, false, false);

  //set target
  this->Reset();

  if(!ros::isInitialized()){
    std::cout << "ros is not initialized" << std::endl;
    int argc = 0;
    char **argv=NULL;
    ros::init(argc,argv,"gazebo", ros::init_options::NoSigintHandler);
  }

  //create ros node 
  this->rosNode.reset(new ros::NodeHandle("gazebo"));

  //request map from move_base
  actor_plan = "/" + this->actor->GetName() + "/move_base/NavfnROS/make_plan";
  sc = this->rosNode->serviceClient<nav_msgs::GetPlan>(actor_plan);

  //request path here
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>("/"+ this->actor->GetName() + "/target_goal", 1,
    boost::bind(&EditedActorPlugin::cmdvel_callback, this, _1), ros::VoidPtr(), &this->rosQueue);

  this->rosSub = this->rosNode->subscribe(so);

  this->rosQueueThread = std::thread(std::bind(&EditedActorPlugin::QueueThread,this));

}

void EditedActorPlugin::cmdvel_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg){

  
  std::cout<<"callback here"<<std::endl;
  x_ = cmd_msg->linear.x;
  y_ = cmd_msg->linear.y;
  if(fabs(this->actor->WorldPose().Pos().X() - x_)> 0.1 || fabs(this->actor->WorldPose().Pos().Y() - y_)>0.1){
  
    this->ChooseNewTarget();
  }  
}
void EditedActorPlugin::QueueThread(){
  static const double timeout = 0.01;
  while(this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void EditedActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = init_pose.Pos();
    
  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void EditedActorPlugin::ChooseNewTarget()
{
  std::unique_lock<std::mutex> lck (target_changed);
  std::cout<<"get new map first for path planning"<<std::endl;
  // map_gen.waitForExistence();
  // if(map_gen.call(empty_nodehandler)){

  // }
  ignition::math::Vector3d newTarget(this->target);
  sc.waitForExistence();
  theplan.request.start.header.stamp = ros::Time::now();
  theplan.request.start.header.frame_id = "map";

  theplan.request.start.pose.position.x = this->actor->WorldPose().Pos().X();
  theplan.request.start.pose.position.y = this->actor->WorldPose().Pos().Y();
  theplan.request.start.pose.position.z = 0;
  theplan.request.start.pose.orientation.x = 0;
  theplan.request.start.pose.orientation.y = 0;
  theplan.request.start.pose.orientation.z = 0;
  theplan.request.start.pose.orientation.w = 1;

  theplan.request.goal.header.stamp=ros::Time::now();
  theplan.request.goal.header.frame_id= "map";
  theplan.request.goal.pose.position.x = x_;
  theplan.request.goal.pose.position.y = y_;
  theplan.request.goal.pose.position.z = 0;
  theplan.request.goal.pose.orientation.x = 0;//this->actor->WorldPose().Rot().X();
  theplan.request.goal.pose.orientation.y = 0;//this->actor->WorldPose().Rot().Y();
  theplan.request.goal.pose.orientation.z = 0;//this->actor->WorldPose().Rot().Z();
  theplan.request.goal.pose.orientation.w = 1;//this->actor->WorldPose().Rot().W();
  theplan.request.tolerance = 0.1;

  if(sc.call(theplan))
  {
    std::cout<<"get newpath and set newtarget" <<std::endl;
    
    if(theplan.response.plan.poses.size() ==0){
      std::cout<<"error no path possible"<<std::endl;
      
    }
    else{
    
      global_path.resize(theplan.response.plan.poses.size());
      for(int i=0; i< theplan.response.plan.poses.size(); i++){

        global_path[i].x=theplan.response.plan.poses[i].pose.position.x;
        global_path[i].y=theplan.response.plan.poses[i].pose.position.y;
      }
      std::cout<<"global path last x/y " <<global_path[theplan.response.plan.poses.size()-1].x << " " <<global_path[theplan.response.plan.poses.size()-1].y <<std::endl;
        double summation_distance =0;
        for(int i=1;i<theplan.response.plan.poses.size();i++){

          if(summation_distance < 0.3){

            summation_distance += sqrt(pow((global_path[i].x - global_path[i-1].x),2.0) + pow((global_path[i].y - global_path[i-1].y),2.0));
            std::cout<<"summation_distance:"<<summation_distance<<std::endl;
          }
          if(i == int(theplan.response.plan.poses.size()-1) || summation_distance >=0.3){
            
            newTarget.X(global_path[i].x);
            newTarget.Y(global_path[i].y);
            
            break;
          }
        }    
    }  
  }
    // newTarget.X(x_);
    // newTarget.Y(y_);
  // while ((newTarget - this->target).Length() < 2.0)
  // {
  //   newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
  //   newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));
    


    // for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    // {
    //   double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
    //       - newTarget).Length();
    //   if (dist < 1.0)
    //   {
    //     newTarget = this->target;
    //     break;
    //   }
    // }
  
  this->target = newTarget;
  std::cout<< "newtarget: " << this->target.X() << " " << this->target.Y() <<std::endl;
}

/////////////////////////////////////////////////
void EditedActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset*= invModelDist;
        _pos-=offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void EditedActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double(); 
  
  if(this->actor->WorldPose().Pos() != this->target){
    std::cout<<"the actor world pose and its target:" << this->actor->WorldPose().Pos() <<": " << this->target << std::endl;
    if((this->actor->WorldPose().Pos().X() - this->target.X())< 0.1 || (this->actor->WorldPose().Pos().Y() - this->target.Y())<0.1){
  
      this->ChooseNewTarget();
    }
  }  
  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
  
  // Choose a new target position if the actor has reached its current
  // target.
  
  //this->ChooseNewTarget();
  //pos = this->target - pose.Pos();
  

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  // this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();
  
  double distance = sqrt(pow(this->target.X() - pose.Pos().X(),2) + pow(this->target.Y() - pose.Pos().Y(),2));
  // std::cout<<"distance:"<<distance<<std::endl;
  if(distance <0.1)
  {
    // std::cout<<"yaw in radian" <<yaw.Radian() << std::endl;
    pose.Pos() = this->target;
    pose.Rot() = this->actor->WorldPose().Rot();
    //pose.Rot()=ignition::math::Quaterniond(1.5707,0,rpy.Z()+yaw.Radian());
  }

  else{

    // Rotate in place, instead of jumping
    if (std::abs(yaw.Radian()) > IGN_DTOR(10))
    {
      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.01);
    }
    else{

      pose.Pos() += pos * this->velocity * dt;
      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
    }
  }

  // Make sure the actor stays within bounds
  // pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  // pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(1.0138);

  //broadcast transform

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Vector3 origin;
  origin.setValue(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  transform.setOrigin(origin);
  tf::Quaternion q; 
  q.setRPY(pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map",this->actor->GetName()+ "_odom"));


  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
