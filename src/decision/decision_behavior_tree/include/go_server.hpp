#pragma once

#ifndef DECISION_BEHAVIOR_TREE__GO_SERVER_HPP_
#define DECISION_BEHAVIOR_TREE__GO_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

class GoServer : public rclcpp::Node
{
private:
    /* data */
public:
    GoServer() : rclcpp::Node("go_server")
    {
        
    }
    ~GoServer();
};



#endif