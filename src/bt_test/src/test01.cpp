#include "btTest/go_goal.hpp"

inline void SleepMS(int ms){
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

BT::NodeStatus CheckBattery(){
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus SaySomething(){
    SleepMS(1000);
    std::cout<<"HIIIIIIIIIIIIIIIII"<<std::endl;
    return BT::NodeStatus::SUCCESS;
  }
/*
1.获取初始坐标
2.获取终点坐标
3.启动导航
4.导航中途识别status为running然后时刻获取坐标
*/
int main()
{
  BT::BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  // factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerSimpleAction("SaySomething", std::bind(SaySomething));

  auto tree = factory.createTreeFromFile("/home/hannah/ws_bt/src/bt_test/src/tree.xml");
 
  // Here, instead of tree.tickWhileRunning(),
  // we prefer our own loop.
  std::cout << "--- ticking\n";
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";

  while(status == BT::NodeStatus::RUNNING) 
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(100));

    std::cout << "--- ticking\n";
    status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";
  }

  return 0;
}