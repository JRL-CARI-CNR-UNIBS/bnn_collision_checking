/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include<iostream>
#include<fstream>
#include <shape_msgs/SolidPrimitive.h>
#include <random>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_distance");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();
  std::ofstream wf("collision.dat", std::ios::out | std::ios::binary);

  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name not defined, use manipulator",pnh.getNamespace().c_str());
    group_name="manipulator";
  }

  std::vector<double> min_range;
  std::vector<double> max_range;
  if (!pnh.getParam("min_range",group_name))
  {
    ROS_ERROR("%s/min_range not defined, use [-2,-2,-2]",pnh.getNamespace().c_str());
    min_range.resize(3,-2);
  }
  if (!pnh.getParam("max_range",group_name))
  {
    ROS_ERROR("%s/max_range not defined, use [+2,+2,+2]",pnh.getNamespace().c_str());
    max_range.resize(3,2);
  }
  assert(min_range.size()==3);
  assert(max_range.size()==3);


  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoaderPtr robot_model_loader=std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  moveit_msgs::GetPlanningScene srv;
  ps_client.call(srv);


  planning_scene::PlanningScene scene(robot_model);
  collision_detection::CollisionRequest req_distance;
  collision_detection::CollisionResult res_distance;



  req_distance.distance=true;
  req_distance.group_name=group_name;
  req_distance.verbose=false;
  req_distance.contacts=false;
  req_distance.cost=false;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[0] = 0.001;

  geometry_msgs::Pose pose_msg;
  pose_msg.orientation.w=1;




  scene.setPlanningSceneMsg(srv.response.scene);
  moveit::core::RobotState state(*move_group.getCurrentState());


  std::random_device rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<double> dis(0.0, 1.0);


  ROS_INFO("start");
  for (int idx=0;idx<100000;idx++)
  {
    moveit_msgs::CollisionObject object;
    object.operation=moveit_msgs::CollisionObject::ADD;
    object.id="probe_sphere";
    object.header.stamp=ros::Time::now();
    object.header.frame_id="world";

    double r;
    r=dis(gen); pose_msg.position.x=min_range.at(0)+(max_range.at(0)-min_range.at(0))*r;
    r=dis(gen); pose_msg.position.y=min_range.at(1)+(max_range.at(1)-min_range.at(1))*r;
    r=dis(gen); pose_msg.position.z=min_range.at(2)+(max_range.at(2)-min_range.at(2))*r;

    object.pose.orientation.w=1.0;
    object.primitive_poses.push_back(pose_msg);
    object.primitives.push_back(primitive);


    if (not scene.processCollisionObjectMsg(object))
    {
      ROS_ERROR("something went wrong");
      return 0;
    }
    for (int idx2=0;idx2<10;idx2++)
    {
      state.setToRandomPositions();
      state.update();
      std::vector<double> configuration;
      state.copyJointGroupPositions(group_name,configuration);
      if (not scene.isStateFeasible(state))
        continue;

      scene.checkCollision(req_distance,res_distance,state);
      double valid=scene.isStateValid(state,group_name)?1.0:0.0;
      configuration.push_back(pose_msg.position.x);
      configuration.push_back(pose_msg.position.y);
      configuration.push_back(pose_msg.position.z);
      configuration.push_back(res_distance.distance); //9
      configuration.push_back(valid); // 12
      wf.write((char*)&configuration[0], configuration.size() * sizeof(double));
    }
  }
  wf.close();
  return 0;
}
