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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_distance");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();


  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name not defined",pnh.getNamespace().c_str());
    return 0;
  }



  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoaderPtr robot_model_loader=std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  moveit_msgs::GetPlanningScene srv;
  ps_client.call(srv);


  planning_scene::PlanningScene scene(robot_model);
  collision_detection::CollisionRequest req_distance;
  collision_detection::CollisionRequest req_simple;
  collision_detection::CollisionResult res_distance;
  collision_detection::CollisionResult res_simple;



  req_distance.distance=true;
  req_distance.group_name=group_name;
  req_distance.verbose=false;
  req_distance.contacts=true;
  req_distance.cost=false;

  req_simple.distance=false;
  req_simple.group_name=group_name;
  req_simple.verbose=false;
  req_simple.contacts=false;
  req_simple.cost=false;


  scene.setPlanningSceneMsg(srv.response.scene);
  moveit::core::RobotState state(*move_group.getCurrentState());

  while (ros::ok())
  {
    state.setToRandomPositions();
    state.update();
    std::vector<double> configuration;
    state.copyJointGroupPositions(group_name,configuration);
    if (not scene.isStateFeasible(state))
      continue;

    scene.checkCollision(req_simple,res_simple,state);
    scene.checkCollision(req_distance,res_distance,state);

    if (scene.isStateValid(state,group_name) && res_distance.distance<0)
    {
    std::cout << "configuration = ";
    for (double& d: configuration )
      std::cout << d << ", ";
    std::cout << "distance = " << res_distance.distance;
    std::cout << ", collison = " << (res_distance.collision?1:0);
    std::cout << ", collison = " << (res_simple.collision?1:0) <<std::endl;
    }
  }

  return 0;
}
