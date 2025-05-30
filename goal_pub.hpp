#ifndef NODES__GOALPUB_HPP
#define NODES__GOALPUB_HPP

#include <behaviortree_cpp/condition_node.h>

#include "aw_decision/msg_type.hpp"
#include "aw_decision/position_2d_type.hpp"

namespace aw_decision
{
    class GoalPub : public BT::SyncActionNode 
    {
        public:
            GoalPub(const std::string &name, const BT::NodeConfiguration &config);

            static BT::PortsList providedPorts();
            BT::NodeStatus tick() override;

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<position_2d_type::Position2D>::SharedPtr publisher_;
    }
}

#endif //! NODES__GOALPUB_HPP