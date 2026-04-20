#!/usr/bin/env python3
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

from moveit.planning import MoveItPy, PlanRequestParameters
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

from std_msgs.msg import Empty
from moveit_msgs.msg import Constraints, OrientationConstraint, CollisionObject, JointConstraint
from geometry_msgs.msg import PoseStamped, Quaternion
import time

def make_full_constraints(
    orientation,
    link_name="tcp_link",
    frame_id="base_link",
    orientation_tolerance=0.5,
    elbow_min=0.1,
    elbow_max=3.14,
):
    """Combine orientation + elbow-up constraints into one Constraints message."""
    constraints = Constraints()

    # Orientation constraint
    oc = OrientationConstraint()
    oc.header.frame_id = frame_id
    oc.link_name = link_name
    oc.orientation = orientation
    oc.absolute_x_axis_tolerance = orientation_tolerance
    oc.absolute_y_axis_tolerance = orientation_tolerance
    oc.absolute_z_axis_tolerance = orientation_tolerance
    oc.weight = 1.0
    constraints.orientation_constraints.append(oc)

    shoulder_pan_min, shoulder_pan_max = -3.14, 3.14
    # Shoulder joint constraint
    jc1 = JointConstraint()
    jc1.joint_name = "shoulder_pan_joint"
    jc1.position = (shoulder_pan_min + shoulder_pan_max) / 2.0
    jc1.tolerance_above = (shoulder_pan_max - shoulder_pan_min) / 2.0
    jc1.tolerance_below = (shoulder_pan_max - shoulder_pan_min) / 2.0
    jc1.weight = 1.0
    constraints.joint_constraints.append(jc1)

    shoulder_lift_min, shoulder_lift_max = -3.14, -1.3
    # Shoulder joint constraint
    jc2 = JointConstraint()
    jc2.joint_name = "shoulder_lift_joint"
    jc2.position = (shoulder_lift_min + shoulder_lift_max) / 2.0
    jc2.tolerance_above = (shoulder_lift_max - shoulder_lift_min) / 2.0
    jc2.tolerance_below = (shoulder_lift_max - shoulder_lift_min) / 2.0
    jc2.weight = 1.0
    constraints.joint_constraints.append(jc2)

    elbow_min, elbow_max = -3.14, 0.1
    # Elbow-up joint constraint
    jc3 = JointConstraint()
    jc3.joint_name = "elbow_joint"
    jc3.position = (elbow_min + elbow_max) / 2.0
    jc3.tolerance_above = (elbow_max - elbow_min) / 2.0
    jc3.tolerance_below = (elbow_max - elbow_min) / 2.0
    jc3.weight = 1.0
    constraints.joint_constraints.append(jc3)

    wrist1_min, wrist1_max = -3.14, 0.1
    # Wrist1 joint constraint
    jc4 = JointConstraint()
    jc4.joint_name = "wrist_1_joint"
    jc4.position = (wrist1_min + wrist1_max) / 2.0
    jc4.tolerance_above = (wrist1_max - wrist1_min) / 2.0
    jc4.tolerance_below = (wrist1_max - wrist1_min) / 2.0
    jc4.weight = 1.0
    # constraints.joint_constraints.append(jc4)

    return constraints

def make_path_constraints(orientation):
    constraints = Constraints()

    # Orientation constraint
    oc = OrientationConstraint()
    oc.header.frame_id = "base_link"
    oc.link_name = "tcp_link"
    oc.orientation = orientation
    oc.absolute_x_axis_tolerance = 0.4
    oc.absolute_y_axis_tolerance = 0.4
    oc.absolute_z_axis_tolerance = 0.4
    oc.weight = 1.0
    constraints.orientation_constraints.append(oc)

    # Joint bounds — keep all joints in unwound ranges
    joint_ranges = {
        # "shoulder_lift_joint": (-3.14, -1.3),  # uncomment if needed
        "elbow_joint":    (-3.14, 0.5),
        "wrist_1_joint":  (-3.14, 0.5),
        # "wrist_2_joint":  (-3.14, 3.14),   # prevent drift beyond ±π
        # "wrist_3_joint":  (-3.14, 3.14),   # prevent drift beyond ±π
    }

    for name, (lo, hi) in joint_ranges.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = (lo + hi) / 2.0
        jc.tolerance_above = (hi - lo) / 2.0
        jc.tolerance_below = (hi - lo) / 2.0
        jc.weight = 0.8
        constraints.joint_constraints.append(jc)

    return constraints


def wait_for_action_server(node, logger, action_name, action_type, timeout=30.0):
    """Block until the action server is available."""
    client = ActionClient(node, action_type, action_name)
    logger.info(f"Waiting for action server '{action_name}'...")
    
    start = time.time()
    while not client.wait_for_server(timeout_sec=1.0):
        elapsed = time.time() - start
        if elapsed > timeout:
            logger.error(f"Timed out waiting for action server '{action_name}'")
            return False
        logger.info(f"Still waiting for '{action_name}'... ({elapsed:.1f}s)")
    
    logger.info(f"Action server '{action_name}' is ready!")
    client.destroy()
    return True

def check_start_state_validity(robot, logger, constraints=None):
    """Print exactly which links MoveIt thinks are in collision."""
    psm = robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        robot_state = scene.current_state
        robot_state.update()  # update transforms

        # Check self collision
        collision_request = robot.get_robot_model()
        result = scene.is_state_valid(robot_state, "ur_manipulator_tcp")

        if not result:
            logger.error("Current state is INVALID according to planning scene")
        else:
            logger.info("Current state is valid")

        # Log current joint positions for debugging
        # joint_names = [
        #     "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        #     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        # ]

        positions = robot_state.joint_positions
        logger.info(f"  Joint positions: {positions} rad")

        # Check each joint against constraints and warn if near boundary
        if constraints is not None:
            for jc in constraints.joint_constraints:
                val = positions.get(jc.joint_name, None)
                if val is None:
                    continue
                lo = jc.position - jc.tolerance_below
                hi = jc.position + jc.tolerance_above
                margin = min(abs(val - lo), abs(val - hi))
                if val < lo or val > hi:
                    logger.error(
                        f"  CONSTRAINT VIOLATED: {jc.joint_name} = {val:.4f} "
                        f"is outside [{lo:.4f}, {hi:.4f}]"
                    )
                elif margin < 0.05:
                    logger.warn(
                        f"  NEAR BOUNDARY: {jc.joint_name} = {val:.4f} "
                        f"is within 0.05 rad of constraint edge [{lo:.4f}, {hi:.4f}]"
                    )


def plan_and_execute(
    robot,
    planning_component,
    logger,
    velocity_scale=0.1,
    acceleration_scale=0.1,
    path_constraints=None,
    planning_time=10.0,       # increase from default 5.0
    planning_attempts=5,      # retry planning this many times
    sleep_time=0.0,
):

    logger.info(f"Planning trajectory (vel_scale={velocity_scale}, accel_scale={acceleration_scale})")

    psm = robot.get_planning_scene_monitor()
    psm.update_frame_transforms()
    with psm.read_write() as scene:
        scene.current_state.update(True)   # force FK update

    planning_component.set_start_state_to_current_state()

    plan_params = PlanRequestParameters(robot, "ompl")
    plan_params.planning_pipeline = "ompl"
    plan_params.planner_id = "APSConfigDefault" # "RRTConnectkConfigDefault"
    plan_params.planning_attempts = planning_attempts
    plan_params.planning_time = planning_time
    plan_params.max_velocity_scaling_factor = velocity_scale
    plan_params.max_acceleration_scaling_factor = acceleration_scale

    plan_result = None
    for attempt in range(planning_attempts):
        logger.info(f"Planning attempt {attempt + 1}/{planning_attempts}")
        check_start_state_validity(robot, logger, constraints=path_constraints)

        # Set constraints fresh each attempt — cleared if attempt fails
        if path_constraints is not None:
            planning_component.set_path_constraints(path_constraints)

        plan_result = planning_component.plan(single_plan_parameters=plan_params)

        # ALWAYS clear constraints, whether planning succeeded or not
        planning_component.set_path_constraints(Constraints())

        if plan_result:
            logger.info(f"Planning succeeded on attempt {attempt + 1}")
            break

        logger.warn(f"Attempt {attempt + 1} failed, retrying...")

        # On repeated failures, progressively relax constraints
        # if attempt == 2 and path_constraints is not None:
        #     logger.warn("Relaxing path constraints after 2 failed attempts...")
        #     path_constraints = relax_constraints(path_constraints, logger)

    # Always clear constraints after planning to avoid affecting next move
    # planning_component.clear_path_constraints()

    if plan_result:
        logger.info("Executing plan")
        plan_result.trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=velocity_scale,
            acceleration_scaling_factor=acceleration_scale,
        )
        robot.execute(plan_result.trajectory, controllers=["joint_trajectory_controller"])
    else:
        logger.error(f"Planning failed after {planning_attempts} attempts")

    time.sleep(sleep_time)

    # plan_result = planning_component.plan(single_plan_parameters=plan_params)

    # if plan_result:
    #     logger.info("Executing plan")
    #     robot.execute(plan_result.trajectory, controllers=["joint_trajectory_controller"])
    # else:
    #     logger.error("Planning failed")

    # time.sleep(sleep_time)

def relax_constraints(constraints, logger, relaxation_factor=2.0):
    """
    Return a copy of constraints with widened tolerances.
    Called automatically after repeated planning failures.
    """
    from copy import deepcopy
    relaxed = deepcopy(constraints)

    for jc in relaxed.joint_constraints:
        jc.tolerance_above *= relaxation_factor
        jc.tolerance_below *= relaxation_factor
        logger.info(f"Relaxed {jc.joint_name}: ±{jc.tolerance_above:.3f} rad")

    for oc in relaxed.orientation_constraints:
        oc.absolute_x_axis_tolerance *= relaxation_factor
        oc.absolute_y_axis_tolerance *= relaxation_factor
        oc.absolute_z_axis_tolerance *= relaxation_factor

    return relaxed




def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # ---------------------------------------------------------------
    # Step 1: Create a temporary node FIRST to probe the action server
    # This must happen before MoveItPy is instantiated
    # ---------------------------------------------------------------
    probe_node = Node("moveit_py_probe")
    
    action_ready = wait_for_action_server(
        node=probe_node,
        logger=logger,
        action_name="/joint_trajectory_controller/follow_joint_trajectory",
        action_type=FollowJointTrajectory,
        timeout=30.0
    )
    
    probe_node.destroy_node()

    if not action_ready:
        logger.error("Action server never became available. Aborting.")
        rclpy.shutdown()
        return

    # ---------------------------------------------------------------
    # Step 2: Small additional buffer AFTER confirming server is up
    # This lets MoveItPy's internal action client connect cleanly
    # ---------------------------------------------------------------
    logger.info("Action server confirmed. Giving MoveItPy time to connect...")
    time.sleep(2.0)

    # ---------------------------------------------------------------
    # Step 3: Now instantiate MoveItPy
    # ---------------------------------------------------------------
    robot = MoveItPy(node_name="moveit_py")
    robot_arm = robot.get_planning_component("ur_manipulator_tcp")
    logger.info("MoveItPy instance created")
    time.sleep(2.0)

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state to current robot state
    # robot_arm.set_start_state_to_current_state()
    logger.info("Using current robot state as start state")


    horizontal_orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
    # full_constraints = make_path_constraints(horizontal_orientation)
    full_constraints = Constraints()
    full_constraints = make_full_constraints(
        orientation=horizontal_orientation,
        link_name="tcp_link",
        elbow_min=-1.57,    # tune these based on your UR5e's
        elbow_max=0.0,   # actual comfortable working range
    )


    # Go home first (no constraint needed here)
    robot_arm.set_start_state_to_current_state()

    # Go to home position
    home_goal = PoseStamped()
    home_goal.header.frame_id = "base_link"
    home_goal.pose.orientation.x = 0.0
    home_goal.pose.orientation.y = 1.0
    home_goal.pose.orientation.z = 0.0
    home_goal.pose.orientation.w = 0.0
    home_goal.pose.position.x = 0.3
    home_goal.pose.position.y = 0.0
    home_goal.pose.position.z = 0.3
    robot_arm.set_goal_state(pose_stamped_msg=home_goal, pose_link="tcp_link")

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 1.0 # 0.93
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0 # 0.38
    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.5
    pose_goal.pose.position.z = 0.3
    robot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tcp_link")

    initial_constraints = Constraints()

    shoulder_pan_min, shoulder_pan_max = -3.14, 3.14
    # Shoulder joint constraint
    jc1 = JointConstraint()
    jc1.joint_name = "shoulder_pan_joint"
    jc1.position = (shoulder_pan_min + shoulder_pan_max) / 2.0
    jc1.tolerance_above = (shoulder_pan_max - shoulder_pan_min) / 2.0
    jc1.tolerance_below = (shoulder_pan_max - shoulder_pan_min) / 2.0
    jc1.weight = 1.0
    initial_constraints.joint_constraints.append(jc1)

    shoulder_lift_min, shoulder_lift_max = -3.14, -1.3
    # Shoulder joint constraint
    jc2 = JointConstraint()
    jc2.joint_name = "shoulder_lift_joint"
    jc2.position = (shoulder_lift_min + shoulder_lift_max) / 2.0
    jc2.tolerance_above = (shoulder_lift_max - shoulder_lift_min) / 2.0
    jc2.tolerance_below = (shoulder_lift_max - shoulder_lift_min) / 2.0
    jc2.weight = 1.0
    initial_constraints.joint_constraints.append(jc2)

    elbow_min, elbow_max = -3.14, 0.1
    # Elbow-up joint constraint
    jc3 = JointConstraint()
    jc3.joint_name = "elbow_joint"
    jc3.position = (elbow_min + elbow_max) / 2.0
    jc3.tolerance_above = (elbow_max - elbow_min) / 2.0
    jc3.tolerance_below = (elbow_max - elbow_min) / 2.0
    jc3.weight = 1.0
    initial_constraints.joint_constraints.append(jc3)

    wrist1_min, wrist1_max = -3.14, 0.1
    # Wrist1 joint constraint
    jc4 = JointConstraint()
    jc4.joint_name = "wrist_1_joint"
    jc4.position = (wrist1_min + wrist1_max) / 2.0
    jc4.tolerance_above = (wrist1_max - wrist1_min) / 2.0
    jc4.tolerance_below = (wrist1_max - wrist1_min) / 2.0
    jc4.weight = 1.0
    # initial_constraints.joint_constraints.append(jc4)

    wrist2_min, wrist2_max = 0, 3.14
    # Wrist2 joint constraint
    jc4 = JointConstraint()
    jc4.joint_name = "wrist_2_joint"
    jc4.position = (wrist2_min + wrist2_max) / 2.0
    jc4.tolerance_above = (wrist2_max - wrist2_min) / 2.0
    jc4.tolerance_below = (wrist2_max - wrist2_min) / 2.0
    jc4.weight = 1.0
    # initial_constraints.joint_constraints.append(jc4)

    oc = OrientationConstraint()
    oc.header.frame_id = "base_link"
    oc.link_name = "tcp_link"
    oc.orientation = horizontal_orientation
    oc.absolute_x_axis_tolerance = 0.4
    oc.absolute_y_axis_tolerance = 0.4
    oc.absolute_z_axis_tolerance = 0.4
    oc.weight = 1.0
    # initial_constraints.orientation_constraints.append(oc)


    plan_and_execute(robot, robot_arm, logger, velocity_scale=0.3, path_constraints=initial_constraints, planning_time=20.0, planning_attempts=10, sleep_time=2.0)

    # Helper to build pose goals cleanly
    def make_pose(x, y, z, ox=0.0, oy=1.0, oz=0.0, ow=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    # 4 positions around the robot, all with orientation constraint
    # waypoints = [
    #     (0.35,  0.0,  0.2),
    #     (-0.35, 0.0,  0.2),
    #     (0.0,   0.35, 0.2),
    #     (0.0,  -0.35, 0.2),
    # ]

    waypoints = [
        (-0.5, 0.5,  0.3),
        (-0.5,  -0.5,  0.3),
        # (-0.4, 0.5,  0.3),
        # (-0.4, -0.3,  0.3),
    ]

    for x, y, z in waypoints:
        # robot_arm.set_start_state_to_current_state()
        robot_arm.set_goal_state(
            pose_stamped_msg=make_pose(x, y, z),
            pose_link="tcp_link"
        )
        plan_and_execute(
            robot, robot_arm, logger,
            velocity_scale=0.3,
            acceleration_scale=0.1,
            path_constraints=full_constraints,
            planning_time=10.0,
            planning_attempts=10,
            sleep_time=2.0,
        )


    logger.info("Sequence Complete.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()