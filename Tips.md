# 各种小妙招集锦
## 使用yaml-cpp解析yaml文件
### 功能包的include
新建yamlsetup.h文件,并更改其内容为
``` bash
#ifndef YAMLSETUP_H
#define YAMLSETUP_H
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif
``` 
在项目的头文件中包含
```bash
#include "yaml-cpp/yaml.h"
#include "yamlsetup.h"
```
### 修改Cmakelists.txt
```bash
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
```
```bash
target_link_libraries(<YOUR_TARGET>
  ${YAMLCPP_LIBRARIES}
)
```
### 修改package.xml
```bash
  <build_depend>yaml-cpp</build_depend>
  <exec_depend>yaml-cpp</exec_depend>
```