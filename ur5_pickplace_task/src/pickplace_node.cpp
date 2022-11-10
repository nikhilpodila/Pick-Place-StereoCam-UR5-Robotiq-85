/**
* \file pickplace_node.cpp 
* \brief Node that performs pick and place with obstacle avoidance
*/
#include <ros/ros.h>
#include<iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ur5_pickplace_task/PickPlaceTask.hpp"


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "pickplace");
    ros::NodeHandle nh("~");
    
    pickplace_task::PickPlaceTask pick_place(nh);
    
    // ROS spinning must run to receive updates from robot
    ros::AsyncSpinner spinner(1);
    spinner.start();
    

    pick_place.execute();
    
    ros::shutdown();
    return 0;
}

