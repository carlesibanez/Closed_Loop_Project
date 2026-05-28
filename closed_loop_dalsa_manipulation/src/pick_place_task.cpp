#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();
  
  void setupPlanningScene();
  void spawnMicroplateInMoveIt();
  
  private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(const geometry_msgs::msg::PoseStamped& target_pose, bool keep_level = false);
  mtc::Task createPickSequence(const geometry_msgs::msg::PoseStamped& pre_grasp_pose);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  
  // Add table collision object
  // moveit_msgs::msg::CollisionObject table;
  // table.id = "table";
  // table.header.frame_id = "base_link";
  // table.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  // shape_msgs::msg::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 1.0;  // x length
  // primitive.dimensions[1] = 1.0;  // y length
  // primitive.dimensions[2] = 0.05; // z height
  
  // geometry_msgs::msg::Pose table_pose;
  // table_pose.position.x = 0.0;
  // table_pose.position.y = 0.0;
  // table_pose.position.z = -0.05;
  // table_pose.orientation.w = 1.0;
  
  // table.primitives.push_back(primitive);
  // table.primitive_poses.push_back(table_pose);
  
  // psi.applyCollisionObject(table);
}

void MTCTaskNode::spawnMicroplateInMoveIt()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject microplate;
  
  // Name matches the '-name' argument in your launch file
  microplate.id = "sbs_microplate"; 
  microplate.header.frame_id = "world"; 
  microplate.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.127, 0.085, 0.015}; 
  
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.35; 
  pose.position.y = 0.0;
  // Use the resting Z height after the physics drop
  pose.position.z = 0.76; 
  pose.orientation.w = 1.0;
  
  microplate.primitives.push_back(primitive);
  microplate.primitive_poses.push_back(pose);
  
  psi.applyCollisionObject(microplate);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}



void MTCTaskNode::doTask()
{
  // Get source loc
  std::vector<double> source_vec;
  node_->get_parameter("source_loc", source_vec);
  RCLCPP_INFO(LOGGER, "Source location: [%f, %f, %f, %f, %f, %f, %f]", source_vec[0], source_vec[1], source_vec[2], source_vec[3], source_vec[4], source_vec[5], source_vec[6]);
  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header.frame_id = "base_link";
  source_pose.pose.position.x = source_vec[0];
  source_pose.pose.position.y = source_vec[1];
  source_pose.pose.position.z = source_vec[2];

  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0); // Roll 180 degrees (Pi radians)
  source_pose.pose.orientation = tf2::toMsg(q);
  // source_pose.pose.orientation.w = source_vec[3];
  // source_pose.pose.orientation.x = source_vec[4];
  // source_pose.pose.orientation.y = source_vec[5];
  // source_pose.pose.orientation.z = source_vec[6];

  // Spawn the microplate in the planning scene so MTC can see it
  this->spawnMicroplateInMoveIt();
  task_ = createPickSequence(source_pose);
  // task_ = createTask(source_pose, false); 

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  RCLCPP_INFO(LOGGER, "Task planning succeeded! Publishing solution to RViz...");
  task_.introspection().publishSolution(*task_.solutions().front());

  // Wait for user to inspect in RViz
  rclcpp::sleep_for(std::chrono::seconds(3));

  RCLCPP_INFO(LOGGER, "Executing task...");
  RCLCPP_INFO(LOGGER, "Task has %zu solutions", task_.solutions().size());

  if (task_.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "No solutions available to execute!");
    return;
  }

  RCLCPP_INFO(LOGGER, "Attempting to execute task...");
  auto result = task_.execute(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "Execution result code: %d", result.val);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution failed with code: %d", result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "Task execution completed successfully!");

  // Now pause for 3 seconds
  RCLCPP_INFO(LOGGER, "Pausing for 3 seconds at source location...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Now go to dest
  // std::vector<double> dest_vec;
  // node_->get_parameter("dest_loc", dest_vec);
  // RCLCPP_INFO(LOGGER, "Dest location: [%f, %f, %f, %f, %f, %f, %f]", dest_vec[0], dest_vec[1], dest_vec[2], dest_vec[3], dest_vec[4], dest_vec[5], dest_vec[6]);
  // geometry_msgs::msg::PoseStamped dest_pose;
  // dest_pose.header.frame_id = "base_link";
  // dest_pose.pose.position.x = dest_vec[0];
  // dest_pose.pose.position.y = dest_vec[1];
  // dest_pose.pose.position.z = dest_vec[2];
  // dest_pose.pose.orientation = tf2::toMsg(q);
  // // dest_pose.pose.orientation.w = dest_vec[3];
  // // dest_pose.pose.orientation.x = dest_vec[4];
  // // dest_pose.pose.orientation.y = dest_vec[5];
  // // dest_pose.pose.orientation.z = dest_vec[6];

  // task_ = createTask(dest_pose, true); // keep_level = true

  // try
  // {
  //   task_.init();
  // }
  // catch (mtc::InitStageException& e)
  // {
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
  //   return;
  // }

  // if (!task_.plan(10))
  // {
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
  //   return;
  // }

  // RCLCPP_INFO(LOGGER, "Task planning succeeded for dest! Publishing solution to RViz...");
  // task_.introspection().publishSolution(*task_.solutions().front());

  // rclcpp::sleep_for(std::chrono::seconds(3)); // optional

  // RCLCPP_INFO(LOGGER, "Executing dest task...");
  // if (task_.solutions().empty()) {
  //   RCLCPP_ERROR(LOGGER, "No solutions available to execute dest!");
  //   return;
  // }

  // auto result2 = task_.execute(*task_.solutions().front());
  // RCLCPP_INFO(LOGGER, "Execution result code for dest: %d", result2.val);
  // if (result2.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
  //   RCLCPP_ERROR(LOGGER, "Execution failed for dest with code: %d", result2.val);
  //   return;
  // }

  RCLCPP_INFO(LOGGER, "Dest task execution completed successfully!");
  return;
}

mtc::Task MTCTaskNode::createTask(const geometry_msgs::msg::PoseStamped& target_pose, bool keep_level)
{
  mtc::Task task;
  task.stages()->setName("ur5e demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_manipulator_tcp";
  const auto& eef_frame = "tcp_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", eef_frame);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);

  // Stage 1: Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  // Stage 2: Move to a safe position above the table
  auto stage_move_to_pose = std::make_unique<mtc::stages::MoveTo>("move to position", sampling_planner);
  stage_move_to_pose->setGroup(arm_group_name);
  stage_move_to_pose->setTimeout(5.0);
  stage_move_to_pose->setIKFrame(eef_frame);
  stage_move_to_pose->setGoal(target_pose);

  /// Conditionally apply the path constraint
  if (keep_level) {
      moveit_msgs::msg::Constraints liquid_constraints;
      liquid_constraints.name = "keep_plate_level";

      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.header.frame_id = "base_link"; 
      ocm.link_name = eef_frame;         

      tf2::Quaternion q_down;
      q_down.setRPY(M_PI, 0.0, 0.0);
      ocm.orientation = tf2::toMsg(q_down);

      ocm.absolute_x_axis_tolerance = 0.1; 
      ocm.absolute_y_axis_tolerance = 0.1; 
      ocm.absolute_z_axis_tolerance = M_PI; 
      ocm.weight = 1.0;

      liquid_constraints.orientation_constraints.push_back(ocm);
      stage_move_to_pose->setPathConstraints(liquid_constraints);
  }

  task.add(std::move(stage_move_to_pose));
  
  setupPlanningScene();
  return task;
}

mtc::Task MTCTaskNode::createPickSequence(const geometry_msgs::msg::PoseStamped& pre_grasp_pose)
{
  mtc::Task task;
  task.stages()->setName("Pick Microplate");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_manipulator_tcp";
  const auto& gripper_group_name = "gripper";
  const auto& eef_frame = "tcp_link";

  // 1. Create Planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1); // Slow for approach
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(0.005); // 5mm steps for high precision

  // ---------------------------------------------------------
  // THE STAGE PIPELINE
  // ---------------------------------------------------------

  // Stage 1: Current State
  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  // Stage 2: Open Gripper
  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  stage_open->setGroup(gripper_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  // Stage 3: Move to Pre-Grasp (OMPL - Free space)
  auto stage_move_pre = std::make_unique<mtc::stages::MoveTo>("move to pre-grasp", sampling_planner);
  stage_move_pre->setGroup(arm_group_name);
  stage_move_pre->setTimeout(5.0);
  stage_move_pre->setIKFrame(eef_frame);
  stage_move_pre->setGoal(pre_grasp_pose);
  task.add(std::move(stage_move_pre));

  // Stage 4: Approach (Cartesian - Drop straight down Z axis)
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach plate", cartesian_planner);
  stage_approach->setGroup(arm_group_name);
  stage_approach->setIKFrame(eef_frame);
  
  geometry_msgs::msg::Vector3Stamped approach_direction;
  approach_direction.header.frame_id = eef_frame; // Move relative to the tool frame
  approach_direction.vector.z = 1.0;              // Move forward along the tool's Z axis
  
  stage_approach->setDirection(approach_direction);
  stage_approach->setMinMaxDistance(0.05, 0.135);  // Drop down between 5cm and 10cm
  task.add(std::move(stage_approach));

  auto stage_allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
  stage_allow_collision->allowCollisions(
      "sbs_microplate",
      task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
      true // True means "allow collisions"
  );
  task.add(std::move(stage_allow_collision));

  // Stage 5: Close Gripper (Position control for plate width)
  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
  stage_close->setGroup(gripper_group_name);
  stage_close->setGoal("closed_short"); // Use the SRDF state that matches the plate width
  task.add(std::move(stage_close));

  // Stage 6: Attach Plate to Planning Scene
  auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach plate");
  stage_attach->attachObject("sbs_microplate", eef_frame); // Name must match Gazebo/MoveIt object ID
  std::vector<std::string> touch_links = 
      task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry();
  // touch_links.push_back("finger2");
  stage_attach->allowCollisions("sbs_microplate", touch_links, true);
  task.add(std::move(stage_attach));

  // Stage 7: Lift (Cartesian - Straight back up)
  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("lift plate", cartesian_planner);
  stage_lift->setGroup(arm_group_name);
  stage_lift->setIKFrame(eef_frame);
  
  geometry_msgs::msg::Vector3Stamped lift_direction;
  lift_direction.header.frame_id = "base_link"; // Move relative to the world
  lift_direction.vector.z = 1.0;                // Straight up in the world Z
  
  stage_lift->setDirection(lift_direction);
  stage_lift->setMinMaxDistance(0.05, 0.10);
  task.add(std::move(stage_lift));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  options.append_parameter_override("use_sim_time", true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  rclcpp::sleep_for(std::chrono::seconds(4));

  mtc_task_node->doTask();

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
