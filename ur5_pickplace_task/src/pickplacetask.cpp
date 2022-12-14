

#include "ur5_pickplace_task/PickPlaceTask.hpp"

namespace pickplace_task {
    
    PickPlaceTask::PickPlaceTask(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {
        
        // Read the required parameters for the task
        readParams();
        ROS_INFO("Done reading parameters!");

        clear_octomap_srv_name_ = "/clear_octomap";
        clear_octomap_srv_client_ = nh.serviceClient<std_srvs::Empty>(clear_octomap_srv_name_);
        
    }
    
    PickPlaceTask::~PickPlaceTask() {}
    
    
    
    void PickPlaceTask::execute() {

        // Wait 5 seconds for loading other elements
        ros::Duration(5.0).sleep();
    
        // Moveit Planning groups for arm and gripper
        static const std::string PLANNING_GROUP_ARM = "ur5_arm";
        static const std::string PLANNING_GROUP_GRIPPER = "gripper";
        moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
        moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    
        // Setting up Robot model and planning scene  
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

        // Initiating object to be picked up
        moveit_msgs::CollisionObject pickup_object;
        pickup_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
        pickup_object.id = "pickup_object";

        // Pick up object size
        shape_msgs::SolidPrimitive pickup_object_primitive;
        pickup_object_primitive.type = pickup_object_primitive.BOX;
        pickup_object_primitive.dimensions.resize(3);
        pickup_object_primitive.dimensions[0] = pickup_obj_size_[0];
        pickup_object_primitive.dimensions[1] = pickup_obj_size_[1];
        pickup_object_primitive.dimensions[2] = pickup_obj_size_[2];
        pickup_object.primitives.push_back(pickup_object_primitive);
    
        // Pick up object pose
        geometry_msgs::Pose pickup_object_pose;
        pickup_object_pose.orientation.w = pickup_obj_orientation_[0];
        pickup_object_pose.orientation.x = pickup_obj_orientation_[1];
        pickup_object_pose.orientation.y = pickup_obj_orientation_[2];
        pickup_object_pose.orientation.z = pickup_obj_orientation_[3];
        pickup_object_pose.position.x = pickup_obj_position_[0];
        pickup_object_pose.position.y = pickup_obj_position_[1];
        pickup_object_pose.position.z = pickup_obj_position_[2];
        pickup_object.primitive_poses.push_back(pickup_object_pose);
        pickup_object.operation = pickup_object.ADD;

        // Apply object collisions to planning scene
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(pickup_object);
        // co2.push_back(cafe_table);

        planning_scene_interface.applyCollisionObjects(collision_objects);

        ROS_INFO_NAMED("pick_place", "Pick-up object added to scene");
        ros::Duration(0.1).sleep();

        // planning_scene_interface.applyCollisionObjects(co2);
        // ros::Duration(0.1).sleep();

        // Enable collisions between pick-up object and Robotiq gripper for grasping
        planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
        collision_detection::AllowedCollisionMatrix& collidableMatrix = ls->getAllowedCollisionMatrixNonConst();
        collidableMatrix.setEntry("pickup_object", "robotiq_85_left_finger_tip_link", true);
        collidableMatrix.setEntry("pickup_object", "robotiq_85_right_finger_tip_link", true);
        moveit_msgs::PlanningScene diff_scene;
        ls->getPlanningSceneDiffMsg(diff_scene);
        planning_scene_interface.applyPlanningScene(diff_scene); 

        // For debugging
        // std::cout << "\nAllowedCollisionMatrix:\n";
        // acm.print(std::cout);
        
        ros::Duration(0.1).sleep();

        //Moveit Initiate plans for ur5_arm and gripper
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    
        // * Step 1: Move ur5_arm to "home" position
    
        move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
        bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place","UR5 arm reaches 'home' %s", success ? "" : "FAILED");
        move_group_interface_arm.move();

        // * Step 2: Move arm above the Pick-up object
    
        // Define arm pose such to be above object
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::Pose target_pose1;
        current_pose = move_group_interface_arm.getCurrentPose("ee_link");
        target_pose1.orientation = current_pose.pose.orientation;
        target_pose1.position.x = pickup_object_pose.position.x;
        target_pose1.position.y = pickup_object_pose.position.y;
        target_pose1.position.z = pickup_object_pose.position.z + step2_z_up_;
    
        // Plan and execute trajectory of Step 2
        move_group_interface_arm.setPoseTarget(target_pose1);
        success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Reached above blue box %s", success ? "" : "FAILED");
        move_group_interface_arm.move();

        // Empty octomap
        clear_octomap_srv_client_.call(empty_srv_call_);
        
        // * Step 3: Open gripper
    
        move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
        success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Opened gripper %s", success ? "" : "FAILED");
        move_group_interface_gripper.move();

        // * Step 4: Lower down arm to pickup position
    
        target_pose1.position.z = target_pose1.position.z - step4_z_down_;
        move_group_interface_arm.setPoseTarget(target_pose1);
        success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Moved down %s", success ? "" : "FAILED");
        move_group_interface_arm.move();

        // * Step 5: Close gripper
    
        // Plan and execute gripper trajectory to grasp
        move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));
        success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Closed Gripper %s", success ? "" : "FAILED");
        move_group_interface_gripper.move();

        // Attach Pick-up object to gripper
        moveit_msgs::AttachedCollisionObject aco;
        aco.object.id = pickup_object.id;
        aco.link_name = "robotiq_85_right_finger_tip_link";
        aco.touch_links.push_back("robotiq_85_left_finger_tip_link");
        aco.object.operation = moveit_msgs::CollisionObject::ADD;
        planning_scene_interface.applyAttachedCollisionObject(aco);

        // * Step 6: Trajectory from pick-up to place (Obstacle avoidance)
    
        // Set target for object to be placed  
        target_pose1.position.z = target_pose1.position.z + dest_z_rel_;
        target_pose1.position.y = dest_y_abs_;
        target_pose1.position.x = target_pose1.position.x + dest_x_rel_;
    
        // Plan and execute trajectory
        move_group_interface_arm.setPoseTarget(target_pose1);
        success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Avoid obstable %s", success ? "" : "FAILED");
        move_group_interface_arm.move();
    
        // ros::Duration(0.1).sleep();

        // * Step 7: Lower the arm to release object
        target_pose1.position.z = target_pose1.position.z - step7_z_down_;
        move_group_interface_arm.setPoseTarget(target_pose1);
        success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Lower arm %s", success ? "" : "FAILED");
        move_group_interface_arm.move();

        // * Step 8: Open gripper to release object
    
        move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
        success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pick_place", "Open and drop object %s", success ? "" : "FAILED");
        move_group_interface_gripper.move();

        // Remove collision of Pick-up object
        ROS_INFO_NAMED("pick_place", "Remove the object from the world");
        std::vector<std::string> object_ids;
        object_ids.push_back(pickup_object.id);
        // object_ids.push_back(cafe_table.id);
        planning_scene_interface.removeCollisionObjects(object_ids);
        ros::Duration(0.1).sleep();


    }

    void PickPlaceTask::readParams() {
        param_keys_ = {"pickup_object/size", "pickup_object/pose/position", "pickup_object/pose/orientation", 
                        "step2_z_up", "step4_z_down", "destination/x_rel", "destination/y_abs", "destination/z_rel",
                        "step7_z_down"};
        for(int param_index = 0; param_index < param_keys_.size(); param_index++) {
            if ( !nh.hasParam(param_keys_[param_index]) ) {
                ROS_ERROR_STREAM("Cannot read " << param_keys_[param_index]);
                ros::requestShutdown();
                return;
            }
        }

        // reading parameters from config - about scene and trajectories
        nh.getParam("pickup_object/size", pickup_obj_size_);
        nh.getParam("pickup_object/pose/position", pickup_obj_position_);
        nh.getParam("pickup_object/pose/orientation", pickup_obj_orientation_);
        nh.getParam("step2_z_up", step2_z_up_);
        nh.getParam("step4_z_down", step4_z_down_);
        nh.getParam("destination/x_rel", dest_x_rel_);
        nh.getParam("destination/y_abs", dest_y_abs_);
        nh.getParam("destination/z_rel", dest_z_rel_);
        nh.getParam("step7_z_down", step7_z_down_);
        ROS_INFO("read singles done");

        
    }
    
}
