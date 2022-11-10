#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <std_srvs/Empty.h>

namespace pickplace_task {
    class PickPlaceTask {
        public:

            PickPlaceTask(ros::NodeHandle& nodeHandle);
            ~PickPlaceTask();
            void execute();
            void readParams();

        private:

            ros::NodeHandle& nh;
            std::vector<std::string> param_keys_;
            std::vector<double> pickup_obj_size_, pickup_obj_position_, pickup_obj_orientation_;
            double step2_z_up_, step4_z_down_, dest_x_rel_, dest_y_abs_, dest_z_rel_, step7_z_down_;

            std_srvs::Empty empty_srv_call_;
            ros::ServiceClient clear_octomap_srv_client_;
            std::string clear_octomap_srv_name_;
		
    };
}