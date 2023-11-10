#include "action/gain_blood_or_bullet_action.hpp"
#include "behaviortree_ros2/plugins.hpp"


namespace decision_behavior_tree
{
    bool GainBloodOrBulletAction::setGoal(RosActionNode::Goal &goal_) 
    {
        auto pos_ = getInput<double>("goal");
        // goal_.goal = pos_.value();
        goal_.goal = 1.0; // 设置目标

        if (goal_.goal != 0)
        {
            std::cout << "成功设置目标点坐标----------" << std::endl;
            return true;
        }
        else
        {
            std::cout << "设置目标点坐标失败----------" << std::endl;
            return false;
        }
    }

    BT::NodeStatus GainBloodOrBulletAction::onResultReceived(const RosActionNode::WrappedResult &wr)
    {
        if(wr.result->done == true)
        std::cout << name().c_str() << ": onResultReceived. Done = true" << std::endl;
        else
        std::cout << name().c_str() << ": onResultReceived. Done = false" << std::endl;

        return wr.result->done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus GainBloodOrBulletAction::onFeedback(const std::shared_ptr<const Feedback> feedback) 
    {
        double now_pos = feedback->current_pos;
        std::cout << "now_pos:" << now_pos << std::endl ;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GainBloodOrBulletAction::onFailure(BT::ActionNodeErrorCode error)
    {
        std::cout << "Error:" << error << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // CreateRosNodePlugin(GainBloodOrBulletAction, "gain_blood_or_bullet"); // 还得配合插件 
    
}// namespace decision_behavior_tree


