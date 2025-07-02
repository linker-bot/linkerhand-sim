# 1. **概述**

灵心巧手，创造万物。

LinkerHand 灵巧手 ROS SDK 是由灵心巧手（北京）科技有限公司开发的一款软件工具，用于驱动其灵巧手系列产品，并提供功能示例。它支持多种设备（如笔记本、台式机、树莓派、Jetson 等），主要服务于人型机器人、工业自动化和科研院所等领域，适用于人型机器人、柔性化生产线、具身大模型训练和数据采集等场景。

# 1.1 **说明**
本程序为LinkerHand制作系列灵巧手Mujoco仿真环境，便于使用者熟悉LinkerHand灵巧手系列产品的使用方式方法，以及进行仿真环境下的模型训练和数据采集

# 2. **使用说明**
```bash
$ mkdir -p Linker_Hand_Mujoco_ros/src    #创建目录
$ cd Linker_Hand_Mujoco_ros/src    #进入目录
$ # 1. 克隆仓库（但不检出文件）
$ git clone --filter=blob:none --no-checkout https://github.com/linkerbotai/linker_hand_sim.git
$ cd linker_hand_sim
# 2. 启用稀疏检出功能
$ git sparse-checkout init --cone
# 3. 指定你要的目录（比如只要 repo 下的 src/ 目录）
$ git sparse-checkout set linker_hand_mujoco_ros/
```
- 修改linker_hand_mujoco_ros/launch/linker_hand_mujoco.launch
根据文件内参数说明修改即可
```bash
$ cd Linker_Hand_Mujoco_ros/
$ catkin_make
$ source ./devel/setup.bash
$ roslaunch linker_hand_mujoco_ros linker_hand_mujoco.launch
```