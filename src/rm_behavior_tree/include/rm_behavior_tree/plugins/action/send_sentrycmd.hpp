#ifndef SEND_SENTRYCMD_HPP
#define SEND_SENTRYCMD_HPP

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "my_msg_interface/msg/sentry_cmd.hpp"

namespace rm_behavior_tree{
class SendSentrycmd : public BT::RosTopicPubNode<my_msg_interface::msg::SentryCmd>
{
public:
    SendSentrycmd(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicPubNode<my_msg_interface::msg::SentryCmd>(instance_name, conf, params){
        blackboard_ = config().blackboard;
    };

    // qos: 1
    bool setMessage(my_msg_interface::msg::SentryCmd &msg) override{
        msg.set__confirm_res(blackboard_->get<uint8_t>("confirmRes"));
        msg.set__confirm_insta_res(blackboard_->get<uint8_t>("confirmInstaRes"));
        msg.set__pending_missile_exch(blackboard_->get<uint16_t>("pendingMissileExch"));
        msg.set__remote_missile_req_count(blackboard_->get<uint8_t>("remoteMissileReqCount"));
        msg.set__remote_health_req_count(blackboard_->get<uint8_t>("remoteHealthReqCount"));
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
};
};// namespace rm_behavior_tree

#endif