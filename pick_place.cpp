/* Author: Tran Ngoc Chuyen */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetStateValidityRequest.h>
#include <moveit_msgs/GetStateValidityResponse.h>

//Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <shape_tools/solid_primitive_dims.h>
#include <string>

#include <moveit_visual_tools/moveit_visual_tools.h>


static const std::string ROBOT_DESCRIPTION="NextageOpen";


int main(int argc, char **argv)
{
  ros::init (argc, argv, "nextage_moveit_planning_execution");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(5).sleep();
  ros::NodeHandle node_handle;

/********************************************/
// MoveGroup, RobotModel Definition
/********************************************/
  moveit::planning_interface::MoveGroup group("right_arm");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  planning_scene::PlanningScene planning_scene(kinematic_model);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Duration(1).sleep();

/********************************************/
// Planner Definition
/********************************************/
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(1.0); 
  
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
 
/********************************************/
// Add Collision Object
/********************************************/
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = group.getEndEffectorLink();

  moveit_msgs::CollisionObject collision_object;

  //Define header, that is used for interpreting the poses
  collision_object.header.frame_id = group.getPlanningFrame();

  collision_object.id = "box1";

  //Define shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.04;
  primitive.dimensions[1] = 0.01;
  primitive.dimensions[2] = 0.2;

  //Define pose
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.3;
  box_pose.position.y = -0.4;
  box_pose.position.z = 0.1;

  //Define collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  //Define attached object
  attached_object.object = collision_object;

  //Add Object to the world
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  ros::Duration(2.0).sleep();

/********************************************/
//set to current state
/********************************************/
  group.setStartStateToCurrentState();

/********************************************/
//Define check service
/********************************************/
  ros::ServiceClient validity_srv = node_handle.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");
  moveit_msgs::GetStateValidity::Request validity_request;
  moveit_msgs::GetStateValidity::Response validity_response;
  validity_request.group_name = "right_arm";

/********************************************/
//Set Pick Goal Pose
/********************************************/
  //Desired pose for the end-effector
  geometry_msgs::Pose target_pose_1;
  target_pose_1.orientation.x = 0.6;
  target_pose_1.orientation.y = -0.5;
  target_pose_1.orientation.z = 0.6;
  target_pose_1.orientation.w = 0.0;
  target_pose_1.position.x = 0.3;;
  target_pose_1.position.y = -0.4;
  target_pose_1.position.z = 0.4;


/********************************************/
//Set Pick Goal State
/********************************************/
  group.setGoalTolerance(0.01);
  ROS_INFO("Set Pick Goal Joints");
  while(1){//begin loop
    if(group.setJointValueTarget(target_pose_1, "RARM_JOINT5_Link")){
      ROS_INFO("OK");
      break;
    }else{
      ROS_INFO("Not OK");
      target_pose_1.orientation.w = target_pose_1.orientation.w + 0.01;
    } 
  }//end of loop
  ros::Duration(1).sleep();


/********************************************/
//Pick Plan
/********************************************/
  ROS_INFO("Pick Plan");
  moveit::planning_interface::MoveGroup::Plan my_plan_pick;
  bool success_pick = group.plan(my_plan_pick);
  if(!success_pick){
     ROS_INFO("Invalid");
     ros::waitForShutdown();
     return 0;
  }

/********************************************/
//Move for piking object
/********************************************/
  ROS_INFO("Move for picking object");
  //bool success_exe = group.execute(my_plan);
  group.asyncMove();
  ros::Duration(15.0).sleep();

  ROS_INFO("Set starting state to current state");
  group.setStartStateToCurrentState();
  ros::Duration(1.0).sleep();

/********************************************/
// Attach object to right hand
/********************************************/
  ROS_INFO("Attach the object to the right hand");
  //group.attachObject(collision_object.id, group.getEndEffectorLink());
  group.attachObject(collision_object.id);
  ros::Duration(12.0).sleep();

/********************************************/
//Generate and Check random Place Goal
/********************************************/
  // Place at random position
  group.startStateMonitor(1.0);
  robot_state::RobotState goal_state_place = *group.getCurrentState();
  goal_state_place.setToRandomPositions(goal_state_place.getJointModelGroup("right_arm"));
  ros::Duration(1).sleep();
  ROS_INFO("Generate and Check Place Goal State");
  while(1){
    robot_state::robotStateToRobotStateMsg(goal_state_place, validity_request.robot_state);
    validity_srv.call(validity_request, validity_response);
    if(validity_response.valid){
      break;
    }else{
      ROS_INFO("Invalid");
      goal_state_place.setToRandomPositions(goal_state_place.getJointModelGroup("right_arm"));
      ros::Duration(1).sleep();
    }
  }

  ROS_INFO("set goal place state");
  group.setJointValueTarget(goal_state_place);
  ros::Duration(1.0).sleep();

/********************************************/
//Place Plan
/********************************************/
  ROS_INFO("Place Plan");
  moveit::planning_interface::MoveGroup::Plan my_plan_place;
  bool success_place = group.plan(my_plan_place);
  if(!success_place){
     ROS_INFO("Invalid");
     ros::waitForShutdown();
     return 0;
  }

/********************************************/
//Move for piking object
/********************************************/
  ROS_INFO("Move for picking object");
  //bool success_exe = group.execute(my_plan);
  group.asyncMove();
  ros::Duration(10.0).sleep();

  group.setStartStateToCurrentState();

  ros::waitForShutdown();
  return 0;
}

