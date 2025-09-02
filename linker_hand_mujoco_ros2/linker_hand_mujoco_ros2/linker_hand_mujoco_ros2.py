
import rclpy,math,sys                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import rclpy.time
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState
import time,threading, json


import sys,os
import threading
from sensor_msgs.msg import JointState
import numpy as np
import mujoco, time
import mujoco.viewer
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
from .utils.mapping import *

JOINT_CONFIG = {
    "L6": {
        "map": L6_JOINT_MAP,
        "arc": L6_JOINT_ARC
    },
    "L7": {
        "map": L7_JOINT_MAP,
        "arc": L7_JOINT_ARC
    },
    "L10": {
        "map": L10_JOINT_MAP,
        "arc": L10_JOINT_ARC
    },
    "L20": {
        "map": L20_JOINT_MAP,
        "arc": L20_JOINT_ARC
    },
    "L21": {
        "map": L21_JOINT_MAP,
        "arc": L21_JOINT_ARC,
    }
}
class MujocoNode(Node):
    def __init__(self):
        super().__init__('linker_hand_mujoco_ros2_node')
        self.declare_parameter("hand_type", "right")
        self.hand_type = self.get_parameter('hand_type').get_parameter_value().string_value
        self.declare_parameter("hand_joint", "L10")
        self.hand_joint = self.get_parameter('hand_joint').get_parameter_value().string_value
        self.create_subscription(JointState,f"/cb_{self.hand_type}_hand_control_cmd",self.hand_cb,10)
        # 直接通过字典获取配置
        joint_config = JOINT_CONFIG.get(self.hand_joint)
        if joint_config:
            self.joint_map = joint_config["map"]
            self.joint_arc = joint_config["arc"]
        else:
            # 处理未匹配的情况（可选）
            self.joint_map = None
            self.joint_arc = None
        XML_PATH = os.path.dirname(os.path.abspath(__file__))+f"/urdf/{self.hand_joint.upper()}/linker_hand_{self.hand_joint.lower()}_{self.hand_type}/linker_hand_{self.hand_joint.lower()}_{self.hand_type}.xml"

        # --- 加载模型 ---
        self.model = mujoco.MjModel.from_xml_path(XML_PATH)
        self.model.dof_damping[:] = 0.8  # 所有关节都设置为 1.0 阻尼
        self.data = mujoco.MjData(self.model)


        print("=" * 20,flush=True)
        print(mujoco.mj_versionString(),flush=True)  # 查看MuJoCo版本
        print("=" * 20,flush=True)
        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        self.model.opt.disableflags = 1
        mujoco.mj_forward(self.model, self.data)

        joint_count = self.model.nu
        joint_names = []
        for i in range(self.model.njnt):
            joint_name = self.model.joint(i).name  # 获取第i个关节的名称
            joint_names.append(joint_name)
            print(f"Joint {i}: {joint_name}",flush=True)
        self.ctrl_values = np.zeros(joint_count)

        # 获取 actuator 控制范围（注意：actuator 不是 joint 本体）
        self.ctrl_ranges = self.model.actuator_ctrlrange.copy()
        sim_thread = threading.Thread(target=self.mujoco_thread)
        sim_thread.start()
        


    # --- MuJoCo 模拟线程 ---
    def mujoco_thread(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            print("MuJoCo viewer running...")
            while viewer.is_running():
                self.data.ctrl[:] = self.ctrl_values
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.001)


    def hand_cb(self,data):
        tmp = []
        position = data.position
        try:
            if self.joint_map is not None:
                if self.hand_type == "left":
                    tmp = range_to_arc_left(position, self.hand_joint)
                elif self.hand_type == "right":
                    tmp = range_to_arc_right(position, self.hand_joint)
                res = self.map_position_array(tmp, self.joint_map)
                self.ctrl_values[:] = res
        except Exception as e:
            self.get_logger().error(f"Error in hand_cb: {e}")
        time.sleep(0.01)  # 确保数据处理的间隔时间



    def map_position_array(self, position, joint_map):
        mapped_array = [0.0] * len(joint_map)  # 初始化20长度的数组
        
        for target_idx, source_idx in joint_map.items():
            if source_idx < len(position):
                mapped_array[target_idx] = position[source_idx]
        
        return mapped_array


def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode()  # 创建 MujocoNode 实例
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
    


if __name__ == '__main__':
    main()
