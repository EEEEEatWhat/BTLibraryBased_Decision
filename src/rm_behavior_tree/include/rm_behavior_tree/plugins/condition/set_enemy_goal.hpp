#ifndef SET_ENEMY_GOAL_HPP
#define SET_ENEMY_GOAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
    class SetEnemyGoal : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    SetEnemyGoal(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&SetEnemyGoal::set_enemy_goal, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    struct Point {
        double x;
        double y;
    };

    Point calculattAimLocation(Point self, Point enemy, double distance) {
        // 敌我之间的斜率
        double grad = (enemy.y - self.y) / (enemy.x - self.x);

        // cos(theta) = grad/adist, So adist = sqrt(grad^2 + 1)
        double adist = sqrt(grad*grad + 1);

        // 它代表的角度值
        double theta = acos(grad / adist); 

        // 检查X减少还是增加（出于四象限计算的原因）
        double xModifier = (enemy.x > self.x) ? 1 : -1;
        double yModifier = (enemy.y > self.y) ? 1 : -1;

        double newX = self.x + xModifier * distance * cos(theta);
        double newY = self.y + yModifier * distance * sin(theta);

        Point newLocation = {newX, newY};
        return newLocation;
    }

    static BT::PortsList providedPorts(){};

    BT::NodeStatus set_enemy_goal(){
        auto enemy_x = blackboard_->get<double>("enemy_x");
        auto enemy_y = blackboard_->get<double>("enemy_y");
        auto my_x = blackboard_->get<double>("my_x");
        auto my_y = blackboard_->get<double>("my_y");
        Point enemy = {enemy_x, enemy_y};
        Point self = {my_x, my_y};
        double tracking_scope = blackboard_->get<double>("tracking_scope");
        Point aimLocation = calculattAimLocation(self, enemy, tracking_scope); 
        geometry_msgs::msg::PoseStamped temp_pose;
        geometry_msgs::msg::Quaternion geo_qnt;
        tf2::Quaternion tf_qnt;
        double temp_yaw;
        tf_qnt.setRPY( 0 , 0 , 0);
        temp_pose.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
        temp_pose.pose.position.set__x(aimLocation.x).set__y(aimLocation.y).set__z(0.0);
        temp_pose.pose.set__orientation(convert<tf2::Quaternion, geometry_msgs::msg::Quaternion>(tf_qnt, geo_qnt, true));
        RCLCPP_INFO(node_->get_logger(),"goal设置成功.为识别到的附近的敌方车... ");
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::SetEnemyGoal>("SetEnemyGoal");
}