# 关于行为树（基于BehaviorTree.CPP v4.5）的编写
## 撰写初衷
### 关于BT库的文档教程有限，可参考的教程包括但不限于
1. https://www.behaviortree.dev/
2. https://github.com/BehaviorTree/BehaviorTree.CPP 源码及其issue（关键）
3. https://github.com/BehaviorTree/BehaviorTree.ROS2 源码及其issue（关键）
4. https://www.google.co.jp/ （google但大概率搜不到什么有用的）

故在此记录本人遇到的一些坑，减少他人踩坑浪费的时间

## Start Here
ps. 先挖点坑有时间就填
### 关于SwitchNode

### 将编写的节点导出为插件并在决策节点中加载
1. 在cpp文件中使用宏(``BT_REGISTER_NODES``或``CreateRosNodePlugin``)将已编写的行为树(action或condition)节点导出为Plugin
2. 修改Cmakelists.txt:
    <br>在add_executable之前将导出的Plugin添加为插件库,例如:<br/>
    ```
    add_library(your_action SHARED 
    plugins/action/your_action.cpp
    )
    list(APPEND plugin_libs your_action)
    ```
    <br>在add_executable之后遍历所有的插件库,为插件库添加依赖(可在添加插件库之前将所有依赖利用set函数统一为${THIS_PACKAGE_DEPS}),控制库的符号导出,将插件库链接到目标,例如:<br/>
    ```
    foreach(bt_plugin ${plugin_libs})
    ament_target_dependencies(${bt_plugin} ${THIS_PACKAGE_DEPS})
    target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
    target_link_libraries(your_target ${bt_plugin})
    endforeach()
    ```
    <br>最后在install中添加所有插件库<br/>
3. 修改.bashrc,将插件库的路径添加到``LD_LIBRARY_PATH``环境变量中,否则会出现类似如下的报错
    ```
    [rm_behavior_tree-1]   what():  Could not load library: libsend_sentrycmd.so: cannot open shared object file: No such file or directory
    ```
    在.bashrc中（一般在最后加一行就行）添加
    ```
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/your/library/
    ```
    其中/path/to/your/library/一般为install/your_tool_pack/lib/your_tool_pack/

    **自启动时的bug**：在.bashrc中添加了上面那句指令，但还是出现上述报错。

    **解决方案**：在自启动指令中先添加上述export指令再启动决策的launch文件。

### 关于StatefulActionNode
