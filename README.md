# BTLibraryBased_Decision（屎山代码开发中...）
- 简介
- 代码框架
- 节点（Node）介绍
- 项目依赖
- 食用说明

## 简介
基于BehaviorTree.CPP(v4.5)和BehaviorTree.ROS2的行为树，为机器人提供智能人工()决策设计

吉林大学TARS_Go战队2024赛季 哨兵决策代码（不完整版）。

## 代码框架

```sh
├── README.md                            // 项目自述文件
└── src
    |   
    ├── rm_decision_interfaces           // 决策相关接口
    |   
    ├── rm_behavior_tree                 // 行为树
    |    |   
    |    ├── config                      // 决策相关的参数配置
    |    |   
    |    └── include                     
    |    |   | 
    |    |   ├── RefereeSystem           // 决策所需的裁判系统头文件
    |    |   | 
    |    |   └── rm_behavior_tree        // 包含行为树节点的相关插件、公共头文件、对裁判系统的客户端
    |    |
    |    ├── launch                      // 行为树的launch文件
    |    |
    |    ├── plugins                     // 实现行为树节点导出为插件
    |    |
    |    ├── src                         // 包含决策节点的cpp文件
    |    |
    |    └── tree                        // 各种行为树
    |    
    ├── Goal_action_server               // 为行为树与nav2的通信提供中间服务端
    |   
    └── lib                              // 主要依赖库
         ├── BehaviorTree.CPP
         |
         └── BehaviorTree.ROS2
```

## 节点（Node）介绍
### decision：
***订阅：***

| 话题| 消息| 描述|

***发布：***

| 话题| 消息| 描述|

***动作：***

| Name| Action| Description|

***参数：***

| Name| 类型| 描述|


啥时候搓完就写


## 项目依赖


## 食用说明
```
mkdir -p decision_ws && cd decision_ws
git clone https://github.com/EEEEEatWhat/BTLibraryBased_Decision.git
colcon build --symlink-install
. install/setup.bash
### 启动BehaviorTreeAction服务端
ros2 launch goal_action_server goalActionServer.launch.py
### 启动决策节点
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py 
