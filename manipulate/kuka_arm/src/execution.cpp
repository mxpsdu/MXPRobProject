
#include <kuka_arm/execution.h>
#include "kuka_arm/Attach.h"
#include "kuka_arm/AttachRequest.h"
#include "kuka_arm/AttachResponse.h" 
TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
  : nh_(nh),
    cycle_counter(0),
    move_group(PLANNING_GROUP),
    eef_group(GRIPPER_GROUP)
{
  /*
   * Setup:
   * Load robot model, robot state, set planning_scene
   * Define the move_group for planning and control purpose
   */
  ros::ServiceClient client = nh_.serviceClient<kuka_arm::Attach>("/link_attacher_node/attach"); 
  ros::ServiceClient client2 = nh_.serviceClient<kuka_arm::Attach>("/link_attacher_node/detach"); 

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  robot_state::RobotState robot_kinematic_state(kinematic_model);


  // set RRT as the planner and set allowed planning time
  move_group.setPlannerId("RRTkConfigDefault");
  move_group.setPlanningTime(10.0);
  eef_group.setPlannerId("RRTkConfigDefault");
  eef_group.setPlanningTime(5.0);

  // Pointer to JointModelGroup for improved performance.
  joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  gripper_joint_model_group =
    eef_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);


  ros::Duration(1.0).sleep();

  while (ros::ok())
  {

    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    // visual_tools.prompt("next step");


    float target_x, target_y, target_z;
    float bin_x, bin_y, bin_z;

    target_x=2.0;
    target_y=0;
    target_z=0.9;
    bin_x=1;
    bin_y=2;
    bin_z=1.3;

    geometry_msgs::Pose target_pose, bin_pose, target_reach;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = target_x - 0.4;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z + 0.1;
    
    target_reach.orientation.w = 1.0;
    target_reach.position.x = target_x - 0.2;
    target_reach.position.y = target_y;
    target_reach.position.z = target_z;

    bin_pose.orientation.w = 1.0;
    bin_pose.position.x = bin_x;
    bin_pose.position.y = bin_y;
    bin_pose.position.z = bin_z;    

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    move_group.setMaxVelocityScalingFactor(0.6);
    eef_group.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Visualizing plan to target: %s",
             success ? "SUCCEEDED" : "FAILED");
    success = move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Moving to pick location: %s",
              success ? "SUCCEEDED" : "FAILED");   
    OpenGripper();
    ros::Duration(1.0).sleep();

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_reach);
    success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Target reach: %s",
             success ? "SUCCEEDED" : "FAILED"); 

    CloseGripper();
    ros::Duration(2.0).sleep();
    kuka_arm::Attach srv;
    srv.request.model_name_1 = "target_box";
    srv.request.link_name_1 = "target_link_1";    
    srv.request.model_name_2 = "kr210";
    srv.request.link_name_2 = "right_gripper_finger_link";  
    client.call(srv);                             
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);
    success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Target retrieval: %s",
             success ? "SUCCEEDED" : "FAILED");  
    // ros::Duration(2.0).sleep();
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(bin_pose);

    success = move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Visualizing plan to drop location: %s",
             success ? "SUCCEEDED" : "FAILED");
    success = move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Moving to drop location: %s",
              success ? "SUCCEEDED" : "FAILED");
    kuka_arm::Attach srv2;
    srv2.request.model_name_1 = "target_box";
    srv2.request.link_name_1 = "target_link_1";    
    srv2.request.model_name_2 = "kr210";
    srv2.request.link_name_2 = "right_gripper_finger_link";  
    client2.call(srv2);  
    OpenGripper();

    ros::Duration(3.0).sleep();
    visual_tools.prompt("next step");
                 
  }
}

bool TrajectorySampler::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.007;  // radians
    gripper_joint_positions[1] = 0.007;  // radians

  }
  else
  {
    gripper_joint_positions[0] = -0.01;  // radians
    gripper_joint_positions[1] = -0.01;  // radians
 
  }

  eef_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = eef_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  return success;
}

bool TrajectorySampler::OpenGripper()
{
  bool success = OperateGripper(false);
  ROS_INFO("Gripper actuation: Opening %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool TrajectorySampler::CloseGripper()
{
  bool success = OperateGripper(true);
  ROS_INFO("Gripper actuation: Closing %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}



TrajectorySampler::~TrajectorySampler() {}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_sampler");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  TrajectorySampler plan_sampler(nh);
  return 0;
}
