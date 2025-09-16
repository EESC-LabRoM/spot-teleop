#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <thread>
#include <atomic>
#include <chrono>
#include <Eigen/Core>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("servo_pose_tracking_spot");

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(
        topic, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) { statusCB(msg); });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("servo_pose_tracking_spot");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    exit(EXIT_FAILURE);
  }

  // Load the planning scene monitor (same setup as demo)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Wait for Planning Scene Monitor to setup
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Create the pose tracker (it will subscribe to target_pose internally)
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  double rot_tol = 0.01;
  const double target_pose_timeout = 0.1;

  // Do NOT publish target_pose from this node.
  // The tracker reads /target_pose (or configured topic) published by another node.

  // Run the pose tracking continuously in a worker thread: relaunch moveToPose whenever it returns.
  std::atomic<bool> running{true};
  std::thread tracking_thread([&tracker, &lin_tol, &rot_tol, &running, &target_pose_timeout] {
    while (running.load() && rclcpp::ok())
    {
      moveit_servo::PoseTrackingStatusCode tracking_status =
          tracker.moveToPose(lin_tol, rot_tol, target_pose_timeout);
      RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                     << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
      // brief pause to avoid tight restart loop
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      // Do not forcibly reset the target pose here; the external publisher controls targets.
    }
  });

  // Clean shutdown handling
  rclcpp::on_shutdown([&]() { running.store(false); });
  tracking_thread.join();

  // Kill executor thread before shutdown
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
