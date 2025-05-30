#include <string>
#include <chrono>
#include <fstream>
#include <filesystem>

#include "aw_decision/msg_type.hpp"
#include "aw_decision/bt_executor.hpp"
#include "aw_decision/nodes/gamestartrewrite.hpp"
#include "aw_decision/nodes/goal_pub.hpp"

namespace aw_decision
{
    BTExecutor::BTExecutor(const rclcpp::NodeOptions &options)
        : rclcpp::Node("sentry_decision_node", options)
    {
        factory_ = std::make_shared<BT::BehaviorTreeFactory>();
        tree_tick_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), // 10Hz
                                                   [this]()
                                                   { auto status = this->tree_.tickOnce();
                                                    if(status == BT::NodeStatus::SUCCESS)
                                                    {
                                                        tree_tick_timer_->cancel();
                                                        RCLCPP_INFO(this->get_logger(), "BTExecutor is finished!");
                                                    } });
    }

    BTExecutor::~BTExecutor() = default;

    void BTExecutor::init()
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "BTExecutor is initializing...");

        /* get static parameter from yaml file */
        getParameter();

        /* register nodes */
        factory_->registerNodeType<aw_decision::GameStart>("GameStart");

        factory_->registerNodeType<aw_decision::GoalPub>("GoalPub");

        if (save_xml_en_)
        {
            /* save behavior tree as xml file */
            saveTreeModel();
        }
        else
        {
            /* register subscription */
            auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            registerSubscription<bt_interfaces::msg::Common>("/common_msg", sub_qos);
            registerSubscription<bt_interfaces::msg::RobotStatus>("/robotstatus_msg", sub_qos);

            /* register tree from xml file */
            RCLCPP_INFO_ONCE(this->get_logger(), "xml file path is %s", xml_file_path_.c_str());
            factory_->registerBehaviorTreeFromFile(xml_file_path_);

            /* create blackboard */
            global_bb_ = BT::Blackboard::create();
            main_bb_ = BT::Blackboard::create(global_bb_);

            /* create behavior tree */
            tree_ = factory_->createTree(tree_id_, main_bb_);

            if (debug_en_)
            {
                rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                cout_logger_ = std::make_shared<BT::StdCoutLogger>(tree_);
                groot2_publisher_ = std::make_shared<BT::Groot2Publisher>(tree_, groot2_port_);

                RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock, 1000, "debug mode is enabled, check Groot2 with port %d.", groot2_port_);
                BT::printTreeRecursively(tree_.rootNode()); // behavior tree hierarchy
                // cout_logger_->enableTransitionToIdle(false); // behavior tree status
                global_bb_->debugMessage(); // behavior tree blackboard
            }
        }
    }

    inline void BTExecutor::getParameter()
    {
        this->declare_parameter<std::string>("xml_file_path", std::string(""));
        this->declare_parameter<std::string>("tree_id", std::string(""));
        this->declare_parameter<bool>("debug_en", false);
        this->declare_parameter<bool>("save_xml_en", false);
        this->declare_parameter<int>("groot2_port", 0);

        this->get_parameter("xml_file_path", xml_file_path_);
        this->get_parameter("tree_id", tree_id_);
        this->get_parameter("debug_en", debug_en_);
        this->get_parameter("save_xml_en", save_xml_en_);
        this->get_parameter("groot2_port", groot2_port_);

        RCLCPP_INFO(this->get_logger(), "xml_file_path: %s,tree_id: %s, debug_en: %s, save_xml_en: %s",
                    xml_file_path_.c_str(), tree_id_.c_str(), std::to_string(debug_en_).c_str(), std::to_string(save_xml_en_).c_str());

        if (!save_xml_en_ && xml_file_path_.empty())
            RCLCPP_ERROR(this->get_logger(), "xml_file_path is empty!");
        if (debug_en_ && groot2_port_ <= 0)
            RCLCPP_ERROR(this->get_logger(), "Invalid Groot2 port!");
    }

    inline void BTExecutor::saveTreeModel()
    {
        RCLCPP_INFO(this->get_logger(), "Saving tree xml...");
        std::string tree_model = BT::writeTreeNodesModelXML(*factory_);
        const std::string save_xml_path = std::filesystem::path(ROOT_DIR) / "bt" / "new_models.xml";
        std::ofstream ofs(save_xml_path);
        if (ofs)
        {
            ofs << tree_model;
            RCLCPP_INFO(this->get_logger(), "xml file have saved successfully! path is: %s", save_xml_path.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "xml file fail to save, please check saved path: %s", save_xml_path.c_str());
            return;
        }
    }

    // template <typename SubscriptionT>
    // inline void BTExecutor::registerSubscription(const std::string &topic_name,
    //                                              const rclcpp::QoS &qos,
    //                                              const std::string &key)
    // {
    //     auto subscription = this->create_subscription<SubscriptionT>(topic_name, qos, [this, key](const typename SubscriptionT::SharedPtr msg)
    //                                                                  { global_bb_->set(key, *msg); });
    //     subscriptions_.push_back(subscription);
    //     if (debug_en_)
    //     {
    //         RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to topic: " << topic_name.c_str());
    //     }
    // }

template <typename SubscriptionT>
inline void BTExecutor::registerSubscription(const std::string& topic_name,
                                          const rclcpp::QoS& qos)
{
    auto subscription = this->create_subscription<SubscriptionT>(
        topic_name,
        qos,
        [this](const typename SubscriptionT::SharedPtr msg) {
            if constexpr (std::is_same_v<SubscriptionT, bt_interfaces::msg::Common>) {
                global_bb_->set("gamestart", msg->gamestart);
                global_bb_->set("stage_remain_time", msg->stage_remain_time);
            }
            else if constexpr (std::is_same_v<SubscriptionT, bt_interfaces::msg::RobotStatus>) {
                //we robot status 
                global_bb_->set("robot_id", msg->robot_id);
                global_bb_->set("current_ammo", msg->current_ammo);

                global_bb_->set("we_hero_hp", msg->we_hero_hp);
                global_bb_->set("current_hp", msg->current_hp   );
                global_bb_->set("we_outpost_hp", msg->we_outpost_hp);
                global_bb_->set("we_base_hp", msg->we_base_hp);
                //enemy status
                global_bb_->set("enemy_hero_hp", msg->enemy_hero_hp);
                global_bb_->set("enemy_sentry_hp", msg->enemy_sentry_hp);
                global_bb_->set("enemy_outpost_hp", msg->enemy_outpost_hp);
                global_bb_->set("enemy_base_hp", msg->enemy_base_hp);
            }
        });
    
    subscriptions_.push_back(subscription);
    
    if (debug_en_) {
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic_name.c_str());
    }
}
} // namespace aw_decision
