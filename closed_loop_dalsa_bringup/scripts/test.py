#!/usr/bin/env python3
import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from std_msgs.msg import Empty
from moveit_msgs.msg import Constraints, OrientationConstraint, CollisionObject
from geometry_msgs.msg import PoseStamped, Quaternion
import time

def main():
    rclpy.init()

    # 1. Initialize MoveItPy
    node = rclpy.create_node("moveit_py")
    logger = get_logger("lab_pick_sequence")
    
    detach_pub = node.create_publisher(Empty, '/detach_plate', 10)

    logger.info("Sending detach signal to Gazebo...")
    time.sleep(1.0) # discovery time
    msg = Empty()
    for _ in range(3):
        detach_pub.publish(msg)
        time.sleep(0.1)

    # 5. Initialize MoveItPy
    # It will look up parameters based on the node_name provided
    bio_robot = MoveItPy(node_name="moveit_py")
    arm = bio_robot.get_planning_component("ur_manipulator_tcp")

    logger.info("Waiting for valid robot state...")
    wait_buffer_ns = 100_000_000 # 0.1 seconds in nanoseconds
    state_received = False

    for _ in range(100): # Allow up to 10s for Gazebo to wake up
        now = node.get_clock().now()
        
        # Safety check: Don't subtract if we are at the very start of the sim
        if now.nanoseconds > wait_buffer_ns:
            current_request_time = now - rclpy.duration.Duration(nanoseconds=wait_buffer_ns)
        else:
            # If sim time is < 0.1s, just ask for the zero-timestamp (latest)
            current_request_time = rclpy.time.Time(nanoseconds=0, clock_type=now.clock_type)

        if bio_robot.get_planning_scene_monitor().wait_for_current_robot_state(current_request_time, 0.1):
            logger.info("Joint states successfully synchronized!")
            state_received = True
            break
        
        rclpy.spin_once(node, timeout_sec=0.1)

    if not state_received:
        logger.error("Could not sync with Gazebo. Is the simulation paused?")
        return
    
    remove_plate_msg = CollisionObject()
    remove_plate_msg.id = "sbs_microplate"
    remove_plate_msg.operation = CollisionObject.REMOVE

    # Use the 'process_collision_object' method
    bio_robot.get_planning_scene_monitor().process_collision_object(remove_plate_msg)
    logger.info("Plate removed from planning scene for approach move.")

    # 2. Define the Horizontal Orientation Constraint
    # This ensures tcp_link stays flat relative to the world frame
    # horiz_constraint = Constraints()
    # oc = OrientationConstraint()
    # oc.header.frame_id = "world"
    # oc.link_name = "tcp_link"
    # oc.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 
    # oc.absolute_x_axis_tolerance = 0.01 # ~0.5 degrees
    # oc.absolute_y_axis_tolerance = 0.01
    # oc.absolute_z_axis_tolerance = 3.14 # Allow 360-degree rotation around vertical Z
    # oc.weight = 1.0
    # horiz_constraint.orientation_constraints.append(oc)

    # Plate Position (World Frame)
    plate_x, plate_y, plate_z = 0.35, 0.0, 0.10

    def move_arm(target_z, description):
        node.get_logger().info(f"Task: {description}")
        
        # 1. Grab the state that we KNOW is valid
        arm.set_start_state_to_current_state()
        verified_state = arm.get_start_state()
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = plate_x
        target_pose.pose.position.y = plate_y
        target_pose.pose.position.z = target_z

        target_pose.pose.orientation.x = 1.0 # Standard "looking down" for UR5e
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        # target_pose.pose.orientation = oc.orientation

        # 2. Tell the PlanningComponent specifically to use this state
        arm.set_start_state(robot_state=verified_state)
        arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="tcp_link") 
        
        # 3. Plan with a longer timeout to allow RRTConnect to work
        # Sometimes RRTConnect needs a second to find a path in tight spaces
        plan_result = arm.plan()
        
        if plan_result:
            node.get_logger().info("Plan found, executing...")
            bio_robot.execute(plan_result.trajectory, controllers=["scaled_joint_trajectory_controller"])
        else:
            node.get_logger().error(f"Failed to plan movement: {description}")

    # 2. Deep-Scan Collision Check
    psm = bio_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        arm.set_start_state_to_current_state()
        is_valid = scene.is_state_valid(
            robot_state=arm.get_start_state(), 
            joint_model_group_name="", # Deep scan
            verbose=True
        )
    
    if not is_valid:
        logger.error("Stopping: Initial state is still in collision!")
        return

    logger.info("Whole-robot state is valid. Starting Motion Sequence...")

    # --- EXECUTION SEQUENCE ---
    # 1. Approach: 10cm above plate
    move_arm(plate_z + 0.10, "APPROACHING PLATE")
    
    # 2. Descend: Exactly to plate height
    # Consider adding 2-3mm offset depending on your finger length
    move_arm(plate_z, "DESCENDING TO GRAB")
    
    # time.sleep(1.0) # Placeholder for gripper activation

    # 3. Lift: Retreat back up
    # move_arm(plate_z + 0.15, "LIFTING PLATE")

    logger.info("Sequence Complete.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()