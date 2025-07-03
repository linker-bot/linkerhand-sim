# 1. **概述**

灵心巧手，创造万物。

LinkerHand 灵巧手 ROS SDK 是由灵心巧手（北京）科技有限公司开发的一款软件工具，用于驱动其灵巧手系列产品，并提供功能示例。它支持多种设备（如笔记本、台式机、树莓派、Jetson 等），主要服务于人型机器人、工业自动化和科研院所等领域，适用于人型机器人、柔性化生产线、具身大模型训练和数据采集等场景。

# 1.1 **说明**
本程序为LinkerHand制作系列灵巧手Mujoco仿真环境，便于使用者熟悉LinkerHand灵巧手系列产品的使用方式方法，以及进行仿真环境下的模型训练和数据采集

# 2. **使用说明**
```bash
$ mkdir -p Linker_Hand_Mujoco_ros/src    #创建目录
$ cd Linker_Hand_Mujoco_ros/src    #进入目录
$ # 1. 克隆仓库（使用 sparse 模式 + blob 过滤，节省空间）
$ git clone --filter=blob:none --sparse https://github.com/linkerbotai/linker_hand_sim.git
$ # 2. 进入仓库目录
$ cd linker_hand_sim
$ # 3. 设置 sparse-checkout 目录
$ git sparse-checkout set linker_hand_mujoco_ros
$ cd Linker_Hand_Mujoco_ros/src/linker_hand_sim/
$ pip install -r requirements.txt
```
- 修改linker_hand_mujoco_ros/launch/linker_hand_mujoco.launch
根据文件内参数说明修改即可
```bash
$ cd Linker_Hand_Mujoco_ros/
$ catkin_make
$ source ./devel/setup.bash
$ roslaunch linker_hand_mujoco_ros linker_hand_mujoco.launch
```

# 3. **topic说明**
- /cb_right_hand_control_cmd or /cb_left_hand_control_cmd
```bash
rostopic pub /cb_right_hand_control_cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['']
position: [200,200,200,200,200,200,200,200,200,200]
velocity: [0]
effort: [0]"
```
- position 说明
  L7:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]

  L10: ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]

  L20: ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]

  L21: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

  L25: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

# 3.1 **GUI控制**
可以使用 Linker_Hand_SDK的[gui_control](https://github.com/linkerbotai/linker_hand_sdk/blob/main/README_CN.md)控制仿真环境
