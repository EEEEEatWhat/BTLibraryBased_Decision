#ifndef DECISION_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP
#define DECISION_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

namespace decision_behavior_tree
{
    enum class BtStatus { SUCCEEDED, FAILED, CANCELED };
    
    class BehaviorTreeEngine
    {
    public:
        explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
        virtual ~BehaviorTreeEngine(){};

          /**
            * @brief Function to execute a BT at a specific rate
            * @param tree BT to execute
            * @param onLoop Function to execute on each iteration of BT execution
            * @param cancelRequested Function to check if cancel was requested during BT execution
            * @param loopTimeout Time period for each iteration of BT execution
            * @return nav2_behavior_tree::BtStatus Status of BT execution
            */
        BtStatus run(
            BT::Tree * tree,
            std::function<void()> onLoop,
            std::function<bool()> cancelRequested,
            std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

          /**
            * @brief Function to create a BT from a XML string
            * @param xml_string XML string representing BT
            * @param blackboard Blackboard for BT
            * @return BT::Tree Created behavior tree
            */
        BT::Tree createTreeFromText(
            const std::string & xml_string,
            BT::Blackboard::Ptr blackboard);
        
          /**
            * @brief Function to explicitly reset all BT nodes to initial state
            * @param root_node Pointer to BT root node
            */
        void haltAllActions(BT::TreeNode * root_node);
    protected:
        // The factory that will be used to dynamically construct the behavior tree
        BT::BehaviorTreeFactory factory_;
    };
    
    
} // namespace decision_behavior_tree
#endif // DECISION_BEHAVIOR_TREE_ENGINE_HPP