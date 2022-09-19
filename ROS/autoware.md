# autoware

## 环境配置

> 参考autoware.test.2的md

编译整个项目：`AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`

编译单个package：`AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --package-select autoware_quickstart_examples`

命令行暂停：`ctrl + s`暂停，输入`ctrl + q`继续

