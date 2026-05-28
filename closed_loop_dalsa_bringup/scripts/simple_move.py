import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class URActionClient(Node):
    def __init__(self):
        super().__init__('ur_action_client')
        # Create the action client pointing to the simulation's move_group server
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_pose_goal(self, target_pose: PoseStamped, group_name="ur_manipulator", end_effector_link="tool0"):
        self.get_logger().info('Waiting for move_action server...')
        self._action_client.wait_for_server()

        # 1. Initialize the Goal Message
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        
        # Slow down the 6-axis cobot for safe lab navigation
        req.max_velocity_scaling_factor = 0.1 
        req.max_acceleration_scaling_factor = 0.1

        # 2. Build the Constraints (The "where to go" part)
        # Python requires us to manually define the positional and rotational bounds
        constraint = Constraints()
        constraint.name = "camera_target_pose"

        # Position Constraint (A tiny 1mm sphere at the target location)
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = end_effector_link
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.001] # 1mm tolerance
        bv.primitives.append(sp)
        bv.primitive_poses.append(target_pose.pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraint.position_constraints.append(pos_constraint)

        # Orientation Constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = end_effector_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01 # radians
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0
        constraint.orientation_constraints.append(ori_constraint)

        req.goal_constraints.append(constraint)
        goal_msg.request = req

        # 3. Send the Goal
        self.get_logger().info('Sending 6D pose goal to simulation...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by move_group. Is the pose out of reach?')
            return

        self.get_logger().info('Goal accepted! Robot is planning and moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1: # SUCCESS is 1 in moveit_msgs/MoveItErrorCodes
            self.get_logger().info('Movement successful!')
        else:
            self.get_logger().error(f'Movement failed with error code: {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = URActionClient()

    # --- DEFINE A TEST POSE ---
    # In the future, your OpenCV pipeline will generate these coordinates
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link" # The reference frame
    
    # Example coordinates (modify these to fit safely inside your simulation workspace)
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = -0.2
    target_pose.pose.position.z = 0.2
    
    # Facing downwards (a common orientation for picking plates)
    target_pose.pose.orientation.x = 0.
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    action_client.send_pose_goal(target_pose)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()