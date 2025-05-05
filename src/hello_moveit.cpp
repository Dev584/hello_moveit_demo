#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and cretae the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Spin up a SingleThreadExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0; // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("manipulator")](auto const trajectory){
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  // Define a square in XY plane with side length 0.2m
  std::vector<geometry_msgs::msg::Pose> square_poses;
  double x0 = 0.28, y0 = -0.2, z0 = 0.5;
  double side = 0.2;
  geometry_msgs::msg::Pose p1, p2, p3, p4;
  p1.position.x = x0;
  p1.position.y = y0;
  p1.position.z = z0;

  p2 = p1;
  p2.position.x += side;

  p3 = p2;
  p3.position.y += side;

  p4 = p3;
  p4.position.x -= side;

  // Set fixed orientation
  for (auto*p : {&p1, &p2, &p3, &p4}) {
    p->orientation.w = 1.0;
  }

  square_poses = {p1, p2, p3, p4, p1};

  for (size_t i = 0; i < square_poses.size(); ++i)
  {
    move_group_interface.setPoseTarget(square_poses[i]);

    std::string step_msg = "Planning";
    draw_title(step_msg);
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to plan");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface.plan(plan));

    if (success)
    {
      draw_trajectory_tool_path(plan.trajectory);
      moveit_visual_tools.trigger();
      prompt("Press 'next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning to point %zu failed!", i+1);
      draw_title("planning Failed!");
      moveit_visual_tools.trigger();
      break;
    }

  }

  /* // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Cretate a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  } */

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}