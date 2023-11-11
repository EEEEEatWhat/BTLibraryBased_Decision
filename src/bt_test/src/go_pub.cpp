#include "bt_test/go_pub.hpp"

int main(int argc, char **argv)
{
  // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针。
    rclcpp::spin(std::make_shared<GoPublisher>("GoPublisherNode"));
  // 5.释放资源；
    rclcpp::shutdown();
    return 0;
}