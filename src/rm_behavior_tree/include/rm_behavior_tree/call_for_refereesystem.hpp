#ifndef CALL_FOR_REFEREESYSTEM_HPP
#define CALL_FOR_REFEREESYSTEM_HPP

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "RefereeSystem/DataType.h"

namespace rm_behavior_tree {
    class CallForRefereeSystem
    {
    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;
        uint16_t cmd_id_req;
        bool response_received_;
        rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedPtr client;
    public:
        CallForRefereeSystem(BT::Blackboard::Ptr blackboard) : blackboard_(blackboard) {
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
            client = node_->create_client<my_msg_interface::srv::RefereeMsg>("RequestSerialize");
        };


        rclcpp::Client<my_msg_interface::srv::RefereeMsg>::FutureAndRequestId sendRequest(uint16_t cmd_id_) {
            response_received_ = false;
            while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "waiting for service is interrupted...");
                throw std::runtime_error("Service interrupted");
            }
            RCLCPP_INFO(node_->get_logger(), "waiting for service to appear...");
            }

            auto request = std::make_shared<my_msg_interface::srv::RefereeMsg::Request>();
            request->cmd_id = cmd_id_;
            cmd_id_req = cmd_id_;
            return client->async_send_request(request);
            
        }

        void processResponse(uint16_t cmd_id_) {
            auto response = this->sendRequest(cmd_id_);
            if (rclcpp::spin_until_future_complete(node_,response.future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                // RCLCPP_INFO(node_->get_logger(),"请求正常处理");
                auto result = response.get();
                // RCLCPP_INFO(node_->get_logger(), "response->cmd_id:0x%x", result->cmd_id);
                switch (result->cmd_id) {
                case 0x0001:
                    setGameStatus(result);
                    break;
                case 0x0003:
                    setGameRobotHP(result);
                    break;
                case 0x0101:
                    setPlaygroundEvent(result);
                    break;
                case 0x0102:
                    setExtSupplyProjectileAction(result);
                    break;
                case 0x0201:
                    setRobotStatus(result);
                    break;
                case 0x0203:
                    setRobotPos(result);
                    break;
                case 0x0204:
                    setBuff(result);
                    break;
                case 0x0208:
                    setProgectileAllowance(result);
                    break;
                case 0x0209:
                    setRfidStatus(result);
                    break;
                case 0x020D:
                    setSentryInfo(result);
                    break;
                case 0x0202:
                    setPowerHeatData(result);
                    break;
                default:
                    break;
                }
                // RCLCPP_INFO(node_->get_logger(), "set data in blackboard success!!!");
            } else {
                RCLCPP_INFO(node_->get_logger(),"请求异常");
            }
            response_received_ = true;
        };

        bool checkResponseReceived() {
            return response_received_;
        };

        bool setGameStatus(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::GameStatusStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint8_t>("GameStatusStruct.game_type", struct_.game_type);
                blackboard_->set<uint8_t>("GameStatusStruct.game_progress", struct_.game_progress);
                blackboard_->set<uint16_t>("GameStatusStruct.stage_remain_time", struct_.stage_remain_time);
                blackboard_->set<uint64_t>("GameStatusStruct.SyncTimeStamp", struct_.SyncTimeStamp);
                return true;
            }
            return false;
        };

        bool setGameRobotHP(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::GameRobotHPStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_1_robot_HP", struct_.red_1_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_2_robot_HP", struct_.red_2_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_3_robot_HP", struct_.red_3_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_4_robot_HP", struct_.red_4_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_5_robot_HP", struct_.red_5_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_7_robot_HP", struct_.red_7_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_outpost_HP", struct_.red_outpost_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_base_HP", struct_.red_base_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_1_robot_HP", struct_.blue_1_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_2_robot_HP", struct_.blue_2_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_3_robot_HP", struct_.blue_3_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_4_robot_HP", struct_.blue_4_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_5_robot_HP", struct_.blue_5_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_7_robot_HP", struct_.blue_7_robot_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.blue_base_HP", struct_.blue_outpost_HP);
                blackboard_->set<uint16_t>("GameRobotHPStruct.red_outpost_HP", struct_.blue_base_HP);
                return true;
            }
            return false;
        };

        bool setPlaygroundEvent(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::PlaygroundEventStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint32_t>("PlaygroundEventStruct.event_data", struct_.event_data);
                return true;
            }
            return false;
        };

        bool setExtSupplyProjectileAction(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::ExtSupplyProjectileActionStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint8_t>("ExtSupplyProjectileActionStruct.reserved", struct_.reserved);
                blackboard_->set<uint8_t>("ExtSupplyProjectileActionStruct.supply_robot_id", struct_.supply_robot_id);
                blackboard_->set<uint8_t>("ExtSupplyProjectileActionStruct.supply_projectile_step", struct_.supply_projectile_step);
                blackboard_->set<uint8_t>("ExtSupplyProjectileActionStruct.supply_projectile_num", struct_.supply_projectile_num);
                return true;
            }
            return false;
        };

        bool setPowerHeatData(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::PowerHeatDataStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint16_t>("PowerHeatDataStruct.chassis_voltage", struct_.chassis_voltage);
                blackboard_->set<uint16_t>("PowerHeatDataStruct.chassis_current", struct_.chassis_current);
                blackboard_->set<float>("PowerHeatDataStruct.chassis_power", struct_.chassis_power);
                blackboard_->set<uint16_t>("PowerHeatDataStruct.buffer_energy", struct_.buffer_energy);
                blackboard_->set<uint16_t>("PowerHeatDataStruct.shooter_17mm_1_barrel_heat", struct_.shooter_17mm_1_barrel_heat);
                blackboard_->set<uint16_t>("PowerHeatDataStruct.shooter_17mm_2_barrel_heat", struct_.shooter_17mm_2_barrel_heat);
                blackboard_->set<uint16_t>("PowerHeatDataStruct.shooter_42mm_barrel_heat", struct_.shooter_42mm_barrel_heat);
                return true;
            }
            return false;
        };

        bool setRobotStatus(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::RobotStateStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint8_t>("RobotStateStruct.robot_id", struct_.robot_id);
                blackboard_->set<uint8_t>("RobotStateStruct.robot_level", struct_.robot_level);
                blackboard_->set<uint16_t>("RobotStateStruct.current_HP", struct_.current_HP);
                blackboard_->set<uint16_t>("RobotStateStruct.maximum_HP", struct_.maximum_HP);
                blackboard_->set<uint16_t>("RobotStateStruct.shooter_barrel_cooling_value", struct_.shooter_barrel_cooling_value);
                blackboard_->set<uint16_t>("RobotStateStruct.shooter_barrel_heat_limit", struct_.shooter_barrel_heat_limit);
                blackboard_->set<uint16_t>("RobotStateStruct.chassis_power_limit", struct_.chassis_power_limit);
                blackboard_->set<uint8_t>("RobotStateStruct.power_management_gimbal_output", struct_.power_management_gimbal_output);
                blackboard_->set<uint8_t>("RobotStateStruct.power_management_chassis_output", struct_.power_management_chassis_output);
                blackboard_->set<uint8_t>("RobotStateStruct.power_management_shooter_output", struct_.power_management_shooter_output);
                return true;
            }
            return false;
        };

        bool setRobotPos(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::RobotPositionStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<float>("RobotPositionStruct.x", struct_.x);
                blackboard_->set<float>("RobotPositionStruct.y", struct_.y);
                blackboard_->set<float>("RobotPositionStruct.angle", struct_.angle);
                return true;
            }
            return false;
        };

        bool setBuff(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::RobotBuffStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint8_t>("RobotBuffStruct.recovery_buff", struct_.recovery_buff);
                blackboard_->set<uint8_t>("RobotBuffStruct.cooling_buff", struct_.cooling_buff);
                blackboard_->set<uint8_t>("RobotBuffStruct.defence_buff", struct_.defence_buff);
                blackboard_->set<uint8_t>("RobotBuffStruct.vulnerability_buff", struct_.vulnerability_buff);
                blackboard_->set<uint16_t>("RobotBuffStruct.attack_buff", struct_.attack_buff);
                return true;
            }
            return false;
        };

        bool setProgectileAllowance(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::ProjectileAllowanceStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint16_t>("ProjectileAllowanceStruct.projectile_allowance_17mm", struct_.projectile_allowance_17mm);
                blackboard_->set<uint16_t>("ProjectileAllowanceStruct.projectile_allowance_42mm", struct_.projectile_allowance_42mm);
                blackboard_->set<uint16_t>("ProjectileAllowanceStruct.remaining_gold_coin", struct_.remaining_gold_coin);
                return true;
            }
            return false;
        };

        bool setRfidStatus(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::RobotRfidStateStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint32_t>("RobotRfidStateStruct.rfid_status", struct_.rfid_status);
                return true;
            }
            return false;
        };

        bool setSentryInfo(std::shared_ptr<my_msg_interface::srv::RefereeMsg_Response> result) {
            RM_referee::SentryInfoStruct struct_;
            std::memcpy(&struct_, result->data_stream.data(), static_cast<size_t>(result->data_length));
            if(result->cmd_id == cmd_id_req) {
                blackboard_->set<uint32_t>("SentryInfoStruct.sentry_info", struct_.sentry_info);
                return true;
            }
            return false;
        };


        ~CallForRefereeSystem(){};
    };
}

#endif