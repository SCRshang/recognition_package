#ifndef BT_EXECUTOR_HPP
#define BT_EXECUTOR_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

/***
 * @brief awakelion decision
 */
namespace aw_decision
{
    /***
     * @brief behavior tree executor
     */
    class BTExecutor : public rclcpp::Node
    {
    public:
        explicit BTExecutor(const rclcpp::NodeOptions &options);
        ~BTExecutor();

        /***
         * @brief initialize behavior tree
         */
        void init();

    private:
        /***
         * @brief timer to tick behavior tree
         */
        rclcpp::TimerBase::SharedPtr tree_tick_timer_;

        /***
         * @brief vector to store subscriptions
         */
        std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

        /***
         * @brief behavior tree factory to register existed BT node
         */
        std::shared_ptr<BT::BehaviorTreeFactory> factory_;

        /***
         * @brief global blackboard to store global parameters form subscriptions
         */
        BT::Blackboard::Ptr global_bb_;

        /***
         * @brief maintree blackboard to inherit key-value pairs from global blackboard
         */
        BT::Blackboard::Ptr main_bb_;

        /***
         * @brief whole behavior tree created by factory
         */
        BT::Tree tree_;

        /***
         * @brief logger to output the log info via std::cout
         */
        std::shared_ptr<BT::StdCoutLogger> cout_logger_;

        /***
         * @brief use Groot2 to check tree nodes status
         */
        std::shared_ptr<BT::Groot2er> groot2_publisher_;

        /***
         * @brief behavior tree xml file path
         * @note ONLY for `save_xml_en: false`
         */
        std::string xml_file_path_;

        /***
         * @brief behavior tree id
         */
        std::string tree_id_;

        /***
         * @brief flag whether get debug information
         */
        bool debug_en_;

        /***
         * @brief flag whether save tree nodes as xml file
         */
        bool save_xml_en_;

        /***
         * @brief port for Groot2
         * @note ONLY for `debug_en: true`
         */
        int groot2_port_;

        /***
         * @brief callback function to tick behavior tree
         */
        void treeTick();

        /***
         * @brief get static parameters via config yaml file
         * @note NOTE that this function is ONLY for static parameters, if you want to use dynamic parameters, please use setBlackboard()
         */
        void getParameter();

        /***
         * @brief save behavior tree nodes model as xml file
         */
        void saveTreeModel();

        /***
         * @brief register subscription into `std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_`
         * @tparam SubscriptionT template which is the type of subscription
         * @param topic_name topic name
         * @param qos QoS of subscription
         * @param key key of blackboard entry

         */
        template <typename SubscriptionT>
        void registerSubscription(const std::string &topic_name,
                                  const rclcpp::QoS &qos,
                                  const std::string &key);
    };
} // namespace aw_decision

#endif //! BT_EXECUTOR_HPP