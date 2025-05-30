#include "aw_decision/goal_pub.hpp"
#include "aw_decision/gamestartrewrite.hpp"

namespace aw_decision
{
    GoalPub::GoalPub(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, std::bind(&GoalPub::tick, this), config),
          node_(std::make_shared<rclcpp::Node>("goal_pub_node"))
    {
        publisher_ = node_->create_publisher<position_2d_type::Position2D>("/goal_pose", 10);
    }

    BT::PortsList PublishGoalPose::providedPorts()
    {
        return { 
            BT::InputPort<position_2d_type::Position2D>("goal_pose", "Target pose to publish")
        };
    }

    BT::NodeStatus PublishGoalPose::tick()
    {
        auto goal_pose = getInput<position_2d_type::Position2D>("goal_pose");
        if (!goal_pose) {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input [goal_pose]");
            return BT::NodeStatus::FAILURE;
        }

        publisher_->publish(goal_pose);
        RCLCPP_INFO(
            node_->get_logger(), 
            "Published goal pose: (%.2f, %.2f, %.2f)", 
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            goal_pose.pose.position.z);

        return BT::NodeStatus::SUCCESS;
    }
} // namespace aw_decision