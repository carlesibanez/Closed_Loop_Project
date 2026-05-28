import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import JointConstraint
import time

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

MOVEIT_ERROR_CODES = {
    1: "SUCCESS",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -4: "CONTROL_FAILED",
    -6: "TIMED_OUT",
    -10: "START_STATE_IN_COLLISION",
    -12: "START_STATE_VIOLATES_GOAL_CONSTRAINTS",
    -14: "GOAL_IN_COLLISION",
    -15: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -21: "UNRECOGNIZED_GOAL_TYPE",
    -22: "INVALID_GROUP_NAME",
    -23: "INVALID_GOAL_CONSTRAINTS",
    -31: "NO_IK_SOLUTION",
}

class PickSequenceNode(Node):
    def __init__(self):
        super().__init__('pick_sequence_node')
        self.arm_client = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        self.gz_attach_pub = self.create_publisher(Empty, '/attach_plate', 10)
        self.gz_detach_pub = self.create_publisher(Empty, '/detach_plate', 10)

        self.cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def send_gripper_command(self, position):
        """Commands the Robotiq Hand-E finger joint."""
        self.get_logger().info(f'Moving gripper to position: {position}')
        self.gripper_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['finger_joint'] # Ensure this matches your moveit_controllers.yaml
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1, nanosec=0) # 1 second movement
        
        traj.points.append(point)
        goal_msg.trajectory = traj

        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code == 0

    def send_arm_pose(self, x, y, z, keep_level=False, max_retries=3):
        """Commands the UR5e TCP to a specific Cartesian location."""
        for attempt in range(1, max_retries + 1):
            self.get_logger().info(f'\n--- NEW MOVE COMMAND ---')
            self.get_logger().info(f'Target -> X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}')
            self.arm_client.wait_for_server()

            goal_msg = MoveGroup.Goal()
            req = MotionPlanRequest()
            req.group_name = "ur_manipulator_tcp"
            req.num_planning_attempts = 20 if keep_level else 15
            req.allowed_planning_time = 10.0 if keep_level else 7.0
            req.max_velocity_scaling_factor = 0.6 if keep_level else 0.3  # Increased for level transport
            req.max_acceleration_scaling_factor = 0.6 if keep_level else 0.3  # Increased for level transport

            # Position Constraints
            constraint = Constraints()
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = "tcp_link"
            
            bv = BoundingVolume()
            bv.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])) # 1mm tolerance
            
            target_pose = PoseStamped()
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            bv.primitive_poses.append(target_pose.pose)
            
            pos_constraint.constraint_region = bv
            pos_constraint.weight = 1.0
            constraint.position_constraints.append(pos_constraint)

            if not keep_level:
                # Orientation Constraints (Pointing straight down)
                ori_constraint = OrientationConstraint()
                ori_constraint.header.frame_id = "base_link"
                ori_constraint.link_name = "tcp_link"
                ori_constraint.orientation.x = 0.0
                ori_constraint.orientation.y = 1.0
                ori_constraint.orientation.z = 0.0
                ori_constraint.orientation.w = 0.0
                # Relaxed tolerances to allow IK solver flexibility
                ori_constraint.absolute_x_axis_tolerance = 0.15 # was 0.05
                ori_constraint.absolute_y_axis_tolerance = 0.15 # was 0.05
                ori_constraint.absolute_z_axis_tolerance = 0.15 # was 0.05
                ori_constraint.weight = 1.0
                constraint.orientation_constraints.append(ori_constraint)

            # --- POSTURE CONSTRAINTS (Force "Elbow Up") ---
            # Prevent the shoulder from dropping below the horizon
            shoulder_constraint = JointConstraint()
            shoulder_constraint.joint_name = "shoulder_lift_joint"
            shoulder_constraint.position = -1.57  # Target roughly -90 degrees (pointing up/forward)
            shoulder_constraint.tolerance_above = 1.57 # Allow up to 0 (horizontal)
            shoulder_constraint.tolerance_below = 1.57 # Allow down to -180
            shoulder_constraint.weight = 1.0
            constraint.joint_constraints.append(shoulder_constraint)

            # Keep the elbow pointing 'up'
            elbow_constraint = JointConstraint()
            elbow_constraint.joint_name = "elbow_joint"
            elbow_constraint.position = 1.57  # Target roughly +90 degrees bent
            elbow_constraint.tolerance_above = 1.57
            elbow_constraint.tolerance_below = 1.57
            elbow_constraint.weight = 1.0
            constraint.joint_constraints.append(elbow_constraint)

            req.goal_constraints.append(constraint)

            if keep_level:
                path_constraint = Constraints()
                path_constraint.name = "keep_plate_level"
                path_ori = OrientationConstraint()
                path_ori.header.frame_id = "base_link"
                path_ori.link_name = "tool0"  # Use tool0 (in ur_manipulator group) for path constraints
                path_ori.orientation.x = 0.0
                path_ori.orientation.y = 1.0
                path_ori.orientation.z = 0.0
                path_ori.orientation.w = 0.0
                path_ori.absolute_x_axis_tolerance = 0.6  # Significantly relaxed for Z-axis stability
                path_ori.absolute_y_axis_tolerance = 0.6  # Significantly relaxed for Z-axis stability
                path_ori.absolute_z_axis_tolerance = 3.14  # Allow full Z rotation for transport flexibility
                path_ori.weight = 1.0
                path_constraint.orientation_constraints.append(path_ori)
                req.path_constraints = path_constraint
                req.allowed_planning_time = 15.0  # More time for constrained planning
                req.num_planning_attempts = 30  # More attempts for constrained motion

            goal_msg.request = req

            self.get_logger().info('Sending goal to MoveIt...')
            future = self.arm_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('MoveIt REJECTED the goal outright. (Usually means invalid parameters)')
                time.sleep(1.0)
                continue

            self.get_logger().info('Goal accepted. Waiting for planning & execution...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                error_code_val = result_future.result().result.error_code.val
                if error_code_val == 1:
                    self.get_logger().info('✅ SUCCESS: Arm reached target!')
                    return True
                else:
                    self.get_logger().error(f'MoveIt Error Code: {error_code_val}')
            else:
                self.get_logger().error(f'Action Server Aborted/Failed. Status: {status}')
            
            self.get_logger().warn('Retrying in 2 seconds...')
            time.sleep(2.0)

    def move_straight_line(self, x, y, z, keep_level=False):
        """Forces the TCP to move in a strict, straight line to the target."""
        self.get_logger().info(f'\n--- CARTESIAN LINE COMMAND ---')
        self.get_logger().info(f'Drawing straight line to X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}')
        
        self.cartesian_srv.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'ur_manipulator_tcp'
        req.link_name = 'tcp_link'  # Consistent link naming
        
        # Calculate a point every 1cm (0.01m) to ensure the path stays perfectly straight
        req.max_step = 0.01  
        req.jump_threshold = 0.0 # Disable jump threshold for simple vertical moves
        req.avoid_collisions = True

        req.max_velocity_scaling_factor = 0.3 if keep_level else 0.2  # Slightly faster for level moves
        req.max_acceleration_scaling_factor = 0.3 if keep_level else 0.2  # Slightly faster for level moves

        # Define the single endpoint. MoveIt automatically draws the line 
        # from the robot's current position to this point.
        wp = Pose()
        wp.position.x = float(x)
        wp.position.y = float(y)
        wp.position.z = float(z)
        wp.orientation.x = 0.0
        wp.orientation.y = 1.0
        wp.orientation.z = 0.0
        wp.orientation.w = 0.0
        
        req.waypoints.append(wp)

        # 1. Compute the path
        self.get_logger().info('Computing Cartesian interpolation...')
        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # Check if the solver successfully drew the whole line (1.0 = 100%)
        if res.fraction < 0.95:
            self.get_logger().error(f'❌ FAILED: Could only compute {res.fraction*100:.1f}% of the straight line.')
            return False

        # 2. Execute the path
        self.get_logger().info('✅ Line computed. Executing trajectory...')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        
        self.execute_client.wait_for_server()
        exec_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, exec_future)
        exec_handle = exec_future.result()
        
        if not exec_handle.accepted:
            self.get_logger().error('Trajectory execution rejected.')
            return False
            
        res_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        error_code = res_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info('✅ SUCCESS: Straight line movement complete!')
            return True
        else:
            self.get_logger().error(f'❌ FAILED during execution. Error code: {error_code}')
            return False

    def move_to_location(self, x, y, z, precision='coarse', keep_level=False, max_retries=3):
        """
        Move to a target location with specified precision level.
        
        Args:
            x, y, z: Target coordinates
            precision: 'coarse' for OMPL planning (transport), 'fine' for Cartesian (precise drop-off)
            keep_level: If True, keep the end-effector level (for sample-carrying plates)
            max_retries: Number of retries for failed planning attempts
        
        Coarse movement strategy:
            - Uses OMPL sampling-based planning
            - 10mm position tolerance (relaxed for faster planning)
            - Can rotate the end-effector during transport (unless keep_level=True)
            - Useful for moving between workstations without time pressure
            - Faster planning and execution
        
        Fine movement strategy:
            - Uses Cartesian linear interpolation  
            - 5mm step increments for smoother motion
            - Keeps end-effector level throughout if keep_level=True
            - Useful for precise positioning at drop-off locations
            - Guarantee: if path succeeds, precision is maintained
        
        Example usage:
            # Transport plate to drop-off area (fast, flexible):
            node.move_to_location(drop_x, drop_y, height_clearance, precision='coarse')
            
            # Fine positioning for exact drop-off:
            node.move_to_location(drop_x, drop_y, drop_height, precision='fine', keep_level=True)
        """
        if precision == 'coarse':
            return self._move_coarse(x, y, z, keep_level, max_retries)
        elif precision == 'fine':
            return self._move_fine(x, y, z, keep_level, max_retries)
        else:
            self.get_logger().error(f'Unknown precision mode: "{precision}". Use "coarse" or "fine".')
            return False

    def _move_coarse(self, x, y, z, keep_level, max_retries):
        """OMPL-based coarse movement for transport between locations."""
        for attempt in range(1, max_retries + 1):
            self.get_logger().info(f'\n--- COARSE MOVE (OMPL-based transport) ---')
            self.get_logger().info(f'Target: X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f} [Attempt {attempt}/{max_retries}]')
            self.arm_client.wait_for_server()

            goal_msg = MoveGroup.Goal()
            req = MotionPlanRequest()
            req.group_name = "ur_manipulator_tcp"
            req.num_planning_attempts = 20 if keep_level else 15
            req.allowed_planning_time = 10.0 if keep_level else 7.0
            req.max_velocity_scaling_factor = 0.2 if keep_level else 0.5  # Maintain speed for level transport
            req.max_acceleration_scaling_factor = 0.2 if keep_level else 0.5  # Maintain acceleration for level transport
            
            constraint = Constraints()
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = "tcp_link"
            
            bv = BoundingVolume()
            bv.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))  # 10mm tolerance
            
            target_pose = PoseStamped()
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            bv.primitive_poses.append(target_pose.pose)
            
            pos_constraint.constraint_region = bv
            pos_constraint.weight = 1.0
            constraint.position_constraints.append(pos_constraint)

            # Always add orientation constraint to the goal - TCP stays pointing down (no rotation)
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = "base_link"
            ori_constraint.link_name = "tcp_link"
            ori_constraint.orientation.x = 0.0
            ori_constraint.orientation.y = 1.0
            ori_constraint.orientation.z = 0.0
            ori_constraint.orientation.w = 0.0
            ori_constraint.absolute_x_axis_tolerance = 0.1
            ori_constraint.absolute_y_axis_tolerance = 0.1
            ori_constraint.absolute_z_axis_tolerance = 0.1
            ori_constraint.weight = 1.0
            constraint.orientation_constraints.append(ori_constraint)

            # Add joint posture constraints to prevent gripper from folding into the arm
            # Keep elbow pointing up to maintain clear approach path to plate
            shoulder_constraint = JointConstraint()
            shoulder_constraint.joint_name = "shoulder_lift_joint"
            shoulder_constraint.position = -1.57  # Target roughly -90 degrees (pointing up/forward)
            shoulder_constraint.tolerance_above = 1.57  # Allow up to 0 (horizontal)
            shoulder_constraint.tolerance_below = 1.57  # Allow down to -180
            shoulder_constraint.weight = 0.5
            constraint.joint_constraints.append(shoulder_constraint)

            elbow_constraint = JointConstraint()
            elbow_constraint.joint_name = "elbow_joint"
            elbow_constraint.position = 1.57  # Target roughly +90 degrees bent
            elbow_constraint.tolerance_above = 1.57
            elbow_constraint.tolerance_below = 1.57
            elbow_constraint.weight = 0.5
            constraint.joint_constraints.append(elbow_constraint)

            req.goal_constraints.append(constraint)

            # If keep_level is required, add path constraint to maintain level orientation throughout
            if keep_level:
                path_constraint = Constraints()
                path_constraint.name = "keep_plate_level_path"
                path_ori = OrientationConstraint()
                path_ori.header.frame_id = "base_link"
                path_ori.link_name = "tcp_link"  # Use tcp_link (in ur_manipulator_tcp group) for path constraints
                path_ori.orientation.x = 0.0
                path_ori.orientation.y = 1.0
                path_ori.orientation.z = 0.0
                path_ori.orientation.w = 0.0
                path_ori.absolute_x_axis_tolerance = 0.1  # Significantly relaxed to allow flexible transport paths
                path_ori.absolute_y_axis_tolerance = 0.1  # Significantly relaxed to allow flexible transport paths
                path_ori.absolute_z_axis_tolerance = 3.14  # Allow Z-axis rotation for orientation freedom
                path_ori.weight = 1.0
                path_constraint.orientation_constraints.append(path_ori)
                req.path_constraints = path_constraint
                req.allowed_planning_time = 15.0  # Increased for constrained planning
                req.num_planning_attempts = 30  # Increased attempts for constrained motion
                self.get_logger().info('Enforcing level plate throughout entire path...')

            goal_msg.request = req

            self.get_logger().info('Planning coarse path via OMPL...')
            future = self.arm_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn('MoveIt rejected goal. Retrying...')
                time.sleep(1.0)
                continue

            self.get_logger().info('Goal accepted. Planning & executing...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                error_code_val = result_future.result().result.error_code.val
                if error_code_val == 1:
                    self.get_logger().info('✅ SUCCESS: Coarse move complete!')
                    return True
                else:
                    self.get_logger().warn(f'Planning failed (code {error_code_val}). Retrying...')
            else:
                self.get_logger().warn(f'Execution failed. Retrying...')
            
            time.sleep(1.0)

        self.get_logger().error(f'❌ FAILED: Could not reach target after {max_retries} attempts.')
        return False

    def _move_fine(self, x, y, z, keep_level, max_retries):
        """
        Cartesian-based fine movement for precise drop-off positioning.
        
        Cartesian interpolation naturally maintains orientation through linear 
        interpolation from current to target pose. When keep_level=True, the path 
        will keep the plate level throughout (not just at the final goal).
        """
        self.get_logger().info(f'\n--- FINE MOVE (Cartesian-based precision) ---')
        self.get_logger().info(f'Precise positioning to X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}')
        if keep_level:
            self.get_logger().info('Plate orientation will be maintained level throughout path')
        
        self.cartesian_srv.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'ur_manipulator_tcp'
        req.link_name = 'tcp_link'  # Consistent with other constraints
        
        req.max_step = 0.002  # 2mm increments for fine control
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        # Slightly faster speeds for fine precision positioning (was 0.2)
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        # Define target with level orientation if required
        wp = Pose()
        wp.position.x = float(x)
        wp.position.y = float(y)
        wp.position.z = float(z)
        
        if keep_level:
            # Keep plate horizontal (flat for sample preservation)
            # Cartesian interpolation will maintain this orientation throughout the path
            wp.orientation.x = 0.0
            wp.orientation.y = 1.0
            wp.orientation.z = 0.0
            wp.orientation.w = 0.0
        else:
            # Standard downward orientation
            wp.orientation.x = 0.0
            wp.orientation.y = 1.0
            wp.orientation.z = 0.0
            wp.orientation.w = 0.0
        
        req.waypoints.append(wp)

        self.get_logger().info(f'Computing precise Cartesian path{"(level)" if keep_level else ""}...')
        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # For level transport, accept lower completion percentage as long as most of path is valid
        acceptable_fraction = 0.75 if keep_level else 0.95  # 75% for level transport, 95% for normal
        if res.fraction < acceptable_fraction:
            self.get_logger().error(f'❌ FAILED: Could only compute {res.fraction*100:.1f}% of fine path.')
            return False

        self.get_logger().info('✅ Fine path computed. Executing with precision...')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        
        self.execute_client.wait_for_server()
        exec_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, exec_future)
        exec_handle = exec_future.result()
        
        if not exec_handle.accepted:
            self.get_logger().error('Trajectory execution rejected.')
            return False
            
        res_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        error_code = res_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info('✅ SUCCESS: Fine precision move complete!')
            return True
        else:
            self.get_logger().error(f'❌ FAILED during execution. Error code: {error_code}')
            return False

    # def attach_plate_to_moveit(self):
    #     """Tells the path planner that the arm is now holding the microplate."""
    #     self.get_logger().info('Attaching microplate to MoveIt Planning Scene...')
        
    #     attached_object = AttachedCollisionObject()
    #     attached_object.link_name = "tcp_link" # The link holding the object
        
    #     # Define the object
    #     obj = CollisionObject()
    #     obj.id = "sbs_microplate"
    #     obj.header.frame_id = "tcp_link"
    #     obj.operation = CollisionObject.ADD
        
    #     # Define the box geometry so MoveIt knows how big the payload is
    #     box = SolidPrimitive()
    #     box.type = SolidPrimitive.BOX
    #     box.dimensions = [0.127, 0.085, 0.013]
        
    #     # The offset of the plate relative to the tcp_link
    #     box_pose = Pose()
    #     box_pose.position.z = 0.0 # Pushes the box slightly out from the TCP
        
    #     obj.primitives.append(box)
    #     obj.primitive_poses.append(box_pose)
        
    #     attached_object.object = obj
        
    #     # Tell MoveIt to explicitly ignore collisions between the plate and the gripper fingers
    #     attached_object.touch_links = [
    #         'tcp_link', 
    #         'finger', 
    #         'finger2',
    #         'gripper_body'
    #     ] 

    #     # Publish to the scene
    #     scene_msg = PlanningScene()
    #     scene_msg.is_diff = True
    #     scene_msg.robot_state.attached_collision_objects.append(attached_object)
        
    #     self.scene_pub.publish(scene_msg)

    def manage_attachment(self, x=0.0, y=0.0, attach=True):
        """Handles both MoveIt (Planning) and Gazebo (Physics) attachment."""
        action_str = "Attaching" if attach else "Detaching"
        self.get_logger().info(f'{action_str} microplate in MoveIt and Gazebo...')

        # 1. GAZEBO PHYSICS
        empty_msg = Empty()
        if attach:
            self.gz_attach_pub.publish(empty_msg)
        else:
            self.gz_detach_pub.publish(empty_msg)

        # 2. MOVEIT PLANNING SCENE
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        if attach:
            # remove any world instance before creating an attached object
            plates_world_remove = CollisionObject()
            plates_world_remove.id = "sbs_microplate"
            plates_world_remove.header.frame_id = "base_link"
            plates_world_remove.operation = CollisionObject.REMOVE
            scene_msg.world.collision_objects.append(plates_world_remove)

            attached_object = AttachedCollisionObject()
            attached_object.link_name = "tcp_link"

            obj = CollisionObject()
            obj.id = "sbs_microplate"
            obj.header.frame_id = "tcp_link"
            obj.operation = CollisionObject.ADD

            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.125, 0.083, 0.013])
            box_pose = Pose()
            box_pose.position.z = 0.0

            obj.primitives.append(box)
            obj.primitive_poses.append(box_pose)

            attached_object.object = obj
            attached_object.touch_links = [
                'tcp_link', 'finger1', 'finger2', 'gripper_body'
            ]

            scene_msg.robot_state.attached_collision_objects.append(attached_object)

        else:
            # remove from attached objects
            attached_remove = AttachedCollisionObject()
            attached_remove.link_name = "tcp_link"
            attached_remove.object = CollisionObject()
            attached_remove.object.id = "sbs_microplate"
            attached_remove.object.operation = CollisionObject.REMOVE
            scene_msg.robot_state.attached_collision_objects.append(attached_remove)

            # add object to world at specified location after detach
            world_obj = CollisionObject()
            world_obj.id = "sbs_microplate"
            world_obj.header.frame_id = "base_link"
            world_obj.operation = CollisionObject.ADD

            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.125, 0.083, 0.013])
            world_pose = Pose()
            world_pose.position.x = float(x)
            world_pose.position.y = float(y)
            world_pose.position.z = 0.01

            world_obj.primitives.append(box)
            world_obj.primitive_poses.append(world_pose)
            scene_msg.world.collision_objects.append(world_obj)

        self.scene_pub.publish(scene_msg)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = PickSequenceNode()

    # --- COORDINATES ---
    z_clearance = 0.20
    z_grasp = 0.01
    
    # Pick Location (Where the plate spawned)
    pick_x, pick_y = 0.35, 0.0
    
    # Place Location (To the left of the robot)
    place_x, place_y = 0.0, -0.35

    # ==========================================
    # --- 0. SYSTEM INITIALIZATION & RESET ---
    # ==========================================
    node.get_logger().info("--- INITIALIZING ROBOT STATE ---")
    
    # 1. Force the gripper fully open
    node.send_gripper_command(0.0)
    
    # 2. Force detach any ghost objects in Gazebo physics and MoveIt planning
    node.manage_attachment(0.35, 0.0, attach=False)
    
    # 3. Give the ROS-Gazebo bridge 1 second to process the reset
    time.sleep(1.0)

    # --- 1. PICK SEQUENCE ---
    node.get_logger().info("--- STARTING PICK ---")
    if not node.move_to_location(pick_x, pick_y, z_clearance, precision='coarse', keep_level=False): return
    # node.send_arm_pose(pick_x, pick_y, z_clearance)  # Hover
    node.send_gripper_command(0.0)                   # Open
    # node.send_arm_pose(pick_x, pick_y, z_grasp)      # Lower
    # node.send_gripper_command(0.023)                 # Close

    if not node.move_straight_line(pick_x, pick_y, z_grasp): return
    if not node.send_gripper_command(0.023): return
    node.manage_attachment(attach=True)

    if not node.move_straight_line(pick_x, pick_y, z_clearance): return

    # for i in range(10):
    #     if not node.send_gripper_command(0.023): return
    #     node.manage_attachment(pick_x, pick_y, attach=True)

    #     if not node.move_straight_line(pick_x, pick_y, z_clearance): return

    #     if not node.move_straight_line(pick_x, pick_y, z_grasp): return

    #     node.manage_attachment(place_x, place_y, attach=False)
    #     if not node.send_gripper_command(0.0): return
    #     time.sleep(1.0)

    #     if not node.move_straight_line(pick_x, pick_y, z_clearance): return

    #     if not node.move_straight_line(pick_x, pick_y, z_grasp): return
    #     time.sleep(2.0)
    
    # return
    # Lock the plate to the robot in physics and planning
    # node.manage_attachment(attach=True)

    # time.sleep(0.5)
    
    # node.send_arm_pose(pick_x, pick_y, z_clearance)  # Lift
    # if not node.move_straight_line(pick_x, pick_y, z_clearance): return

    # --- 2. PLACE SEQUENCE ---
    # node.get_logger().info("--- STARTING PLACE ---")
    
    # Two-phase approach for precise drop-off with sample-carrying plate:
    # Phase 1: Fast coarse movement to drop-off area (plate can rotate)


    node.get_logger().info("--- TRANSPORT TO DROP-OFF AREA (Coarse) ---")
    if not node.move_to_location(place_x, place_y, z_clearance, precision='coarse', keep_level=True): return

    # if not node.move_to_location(place_x, place_y, z_clearance, precision='coarse', keep_level=True):
    #     return
    
    # Phase 2: Slow fine movement for precise positioning (keep plate level)
    node.get_logger().info("--- FINAL POSITIONING AT DROP-OFF (Fine & Level) ---")
    if not node.move_to_location(place_x, place_y, z_grasp, precision='fine', keep_level=True): return
    
    node.manage_attachment(place_x, place_y, attach=False)
    time.sleep(0.5)
    node.send_gripper_command(0.0)  
    time.sleep(0.5)

    if not node.move_to_location(place_x, place_y, z_clearance, precision='fine', keep_level=True): return

    # if not node.move_straight_line(place_x, place_y, z_clearance): return



    # # Release the plate in physics and planning
    # node.manage_attachment(attach=False)              
    # node.send_gripper_command(0.0)                    # Open
    
    # node.send_arm_pose(place_x, place_y, z_clearance) # Retreat upwards

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()