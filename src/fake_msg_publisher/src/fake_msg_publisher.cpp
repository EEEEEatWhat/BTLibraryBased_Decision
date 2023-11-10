#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "global_interface/msg/obj_hp.hpp"


using namespace std::chrono_literals;
using ObjHPMsg = global_interface::msg::ObjHP;
using CarPosMsg = global_interface::msg::CarPos;
using GameInfoMsg = global_interface::msg::GameInfo;
using SerialMsg = global_interface::msg::Serial;
using Point2f = global_interface::msg::Point2f;
using ModeSet = global_interface::msg::ModeSet;

class Lier : public rclcpp::Node
{
public:
  Lier() : Node("fake_msg_publisher")
  {
    publisher_CarHP_ = this->create_publisher<ObjHPMsg>("/obj_hp", 10);
    publisher_CarPosMsg_ = this->create_publisher<CarPosMsg>("/car_pos", 10);
    publisher_GameInfoMsg_ = this->create_publisher<GameInfoMsg>("/game_info", 10);
    publisher_SerialMsg_ = this->create_publisher<SerialMsg>("/serial_msg", 10);
    publisher_ModeSet_ = this->create_publisher<ModeSet>("/mode_set", 10);

    auto timer_callback = [this]() -> void { this->timerCallback(); };
    timer_ = this->create_wall_timer(100ms, timer_callback);

    carPos_msg_.pos.resize(12);
    carPos_msg_.pos[5].x = 3.82;
    carPos_msg_.pos[5].y = 7.0;

    for (int i = -1; i < 11; ++i)
    {
      carPos_msg_.pos[i].x = 6.0;
      carPos_msg_.pos[i].y = 4.0;
    }
  }

private:
  void timerCallback()
  {
    makeFake();
    auto temp_pos = std::make_shared<Point2f>();
    temp_pos->x = 3.82;
    temp_pos->y = 7.0;

    auto modeSet_msg = std::make_shared<ModeSet>();
    modeSet_msg->mode = 1;
    modeSet_msg->x = 3.0;
    modeSet_msg->y = 4.0;

    carPos_msg_.pos[5] = *temp_pos;

    publisher_CarHP_->publish(objHP_msg_);
    publisher_CarPosMsg_->publish(carPos_msg_);
    publisher_GameInfoMsg_->publish(gameInfo_msg_);
    publisher_SerialMsg_->publish(serial_msg_);
    publisher_ModeSet_->publish(*modeSet_msg);

    RCLCPP_INFO(this->get_logger(), "Publish Fake Msgs");
  }

  void makeFake()
  {
    objHP_msg_ = std::make_shared<ObjHPMsg>();
    objHP_msg_->header.stamp = this->now();
    objHP_msg_->hp[5] = 500;
    objHP_msg_->hp[6] = 600;
    objHP_msg_->hp[7] = 600;

    for (int i = -1; i < 11; ++i)
    {
      auto temp_pos2 = std::make_shared<Point2f>();
      double aim_x = randomFloat(carPos_msg_.pos[i].x - 0.1, carPos_msg_.pos[i].x + 0.1);
      double aim_y = randomFloat(carPos_msg_.pos[i].y - 0.1, carPos_msg_.pos[i].y + 0.1);

      if (aim_x < 0.0)
        aim_x = 0.0;
      if (aim_x > w_)
        aim_x = w_;
      if (aim_y < 0.0)
        aim_y = 0.0;
      if (aim_y > h_)
        aim_y = h_;

      temp_pos2->x = aim_x;
      temp_pos2->y = aim_y;
      carPos_msg_.pos[i] = *temp_pos2;
    }

    carPos_msg_.header.stamp = this->now();

    gameInfo_msg_ = std::make_shared<GameInfoMsg>();
    gameInfo_msg_->header.stamp = this->now();
    gameInfo_msg_->game_stage = 4;

    serial_msg_ = std::make_shared<SerialMsg>();
    serial_msg_->header.stamp = this->now();
  }

  double randomFloat(double min, double max)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
  }

  rclcpp::Publisher<ObjHPMsg>::SharedPtr publisher_CarHP_;
  rclcpp::Publisher<CarPosMsg>::SharedPtr publisher_CarPosMsg_;
  rclcpp::Publisher<GameInfoMsg>::SharedPtr publisher_GameInfoMsg_;
  rclcpp::Publisher<SerialMsg>::SharedPtr publisher_SerialMsg_;
  rclcpp::Publisher<ModeSet>::SharedPtr publisher_ModeSet_;
  rclcpp::TimerBase::SharedPtr timer_;

  ObjHPMsg::SharedPtr objHP_msg_;
  CarPosMsg carPos_msg_;
  GameInfoMsg::SharedPtr gameInfo_msg_;
  SerialMsg::SharedPtr serial_msg_;

  double w_ = 12.0;
  double h_ = 8.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lier>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}