#!/usr/bin/env python3
import rclpy, os, threading, time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from .utils.mapping import *
L7_MAPPING = {
    0: (6, 1),
    1: (1, 1),
    2: (0, 1),
    3: (0, 3),
    4: (0, 2),
    5: (0, 2),
    7: (2, 1),
    8: (2, 1),
    9: (2, 1),
    11: (3, 1),
    12: (3, 1),
    13: (3, 1),
    16: (4, 1),
    17: (4, 1),
    18: (4, 1),
    21: (5, 1),
    22: (5, 1),
    23: (5, 1)
}
L10_MAPPING = {
    0: 9,    # 特殊关节
    1: 1,    # 拇指
    2: 0,    # 食指基关节
    3: 0,    # 食指中关节
    4: 0,    # 食指末关节
    6: 6,    # 特殊关节
    7: 2,    # 中指基关节
    8: 2,    # 中指中关节
    9: 2,    # 中指末关节
    11: 3,   # 无名指基关节
    12: 3,   # 无名指中关节
    13: 3,   # 无名指末关节
    15: 7,   # 特殊关节
    16: 4,   # 小指基关节
    17: 4,   # 小指中关节
    18: 4,   # 小指末关节
    20: 8,   # 特殊关节
    21: 5,   # 额外关节1
    22: 5,   # 额外关节2
    23: 5    # 额外关节3
}
L20_MAPPING = {
    0: 0, 7: 1, 12: 2, 17: 3, 22: 4,
    1: 5, 6: 6, 11: 7, 16: 8, 21: 9,
    2: 10, 3: 15, 8: 16, 13: 17, 18: 18, 23: 19
}
L21_MAPPING = {
    0: 6,  1: 1,   2: 21,   
    3: 7,  4: 2,  5: 22,
    6: 8, 7: 3,   8: 23,
    9: 9,  10: 4, 11: 24,
    12: 10, 13: 5, 
    14: 0,  15: 15, 16: 20
}
class LinkerHandPyBulletNode(Node):
    def __init__(self):
        super().__init__('linker_hand_pybullet_ros2_node')
        self.declare_parameter('hand_joint', 'L7')
        #self.hand_type = "right"
        self.hand_joint = self.get_parameter('hand_joint').value
        self.left_hand_state_pub = self.create_publisher(JointState, '/cb_left_hand_state_sim', 10)
        self.right_hand_state_pub = self.create_publisher(JointState, '/cb_right_hand_state_sim', 10)
        self.create_subscription(JointState,f"/cb_left_hand_control_cmd",self.left_hand_cb,10)
        self.create_subscription(JointState,f"/cb_right_hand_control_cmd",self.right_hand_cb,10)
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.init_sim()
        self.pub_threading = threading.Thread(target=self.pub_hand_state, daemon=True)
        self.pub_threading.start()
        self.sim_threading = threading.Thread(target=self.sim_controller.showSim, daemon=True)
        self.sim_threading.start()
    

    def init_sim(self):
        urdf_path_left = f"{self.current_dir}/urdf/{self.hand_joint.lower()}/left/linkerhand_{self.hand_joint.lower()}_left.urdf"
        urdf_path_right = f"{self.current_dir}/urdf/{self.hand_joint.lower()}/right/linkerhand_{self.hand_joint.lower()}_right.urdf"
        if self.hand_joint == "L7":
            from linker_hand_pybullet_ros2.utils.l7_sim_controller import L7SimController
            self.sim_controller = L7SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L10":
            from linker_hand_pybullet_ros2.utils.l10_sim_controller import L10SimController
            self.sim_controller = L10SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L20":
            from linker_hand_pybullet_ros2.utils.l20_sim_controller import L20SimController
            urdf_path_left = f"{self.current_dir}/urdf/{self.hand_joint.lower()}/linker_hand_l20_left/linker_hand_{self.hand_joint.lower()}_left.urdf"
            print(urdf_path_left)
            urdf_path_right = f"{self.current_dir}/urdf/{self.hand_joint.lower()}/linker_hand_l20_right/linker_hand_{self.hand_joint.lower()}_right.urdf"
            self.sim_controller = L20SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L21":
            from linker_hand_pybullet_ros2.utils.l21_sim_controller import L21SimController
            self.sim_controller = L21SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)

        

    def pub_hand_state(self):
        tmp_pose = [0.00] * 25
        while True:
            if len(self.sim_controller.left_hand_current_position) == 0:
                #self.get_logger().info("Left hand position is empty, waiting for data...")
                pass
            else:
                # TODO: 这里返回的是仿真关节数，需要转换为topic发布的关节数
                left_pose = self.sim_controller.left_hand_current_position
                if self.hand_joint == "L21":
                    tmp_pose = [0.00] * 25
                    items = L21_MAPPING.items()
                    for target_idx, source_idx in items:
                        tmp_pose[source_idx] = left_pose[target_idx]
                    left_state_msg = self.joint_msg(hand="left",position=arc_to_range_left(hand_arc_l=tmp_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.left_hand_state_pub.publish(left_state_msg)
                else:
                    left_state_msg = self.joint_msg(hand="left",position=arc_to_range_left(hand_arc_l=left_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.left_hand_state_pub.publish(left_state_msg)
            if len(self.sim_controller.right_hand_current_position) == 0:
                #self.get_logger().info("Right hand position is empty, waiting for data...")
                pass
            else:
                right_pose = self.sim_controller.right_hand_current_position
                if self.hand_joint == "L21":
                    tmp_pose = [0.00] * 25
                    items = L21_MAPPING.items()
                    for target_idx, source_idx in items:
                        tmp_pose[source_idx] = right_pose[target_idx]
                    right_state_msg = self.joint_msg(hand="right",position=arc_to_range_right(right_arc=tmp_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.right_hand_state_pub.publish(right_state_msg)
                else:
                    right_state_msg = self.joint_msg(hand="right",position=arc_to_range_right(right_arc=right_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.right_hand_state_pub.publish(right_state_msg)
            time.sleep(0.1)

    def left_hand_cb(self, msg):
        left_hand_pos = [0] * 25
        if len(msg.position) > 0 :
            if self.hand_joint == "L7":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L7")
                for target_idx, (source_idx, multiplier) in L7_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx] * multiplier
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L10":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L10")
                for target_idx, source_idx in L10_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L20":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L20")
                for target_idx, source_idx in L20_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L21":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L21")
                for target_idx, source_idx in L21_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
    
    def right_hand_cb(self, msg):
        right_hand_pos = [0] * 25
        if self.hand_joint == "L7":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L7")
            for target_idx, (source_idx, multiplier) in L7_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx] * multiplier
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L10":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L10")
            for target_idx, source_idx in L10_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx]
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L20":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L20")
            for target_idx, source_idx in L20_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx]
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L21":
                cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L21")
                for target_idx, source_idx in L21_MAPPING.items():
                    right_hand_pos[target_idx] = cmd_right_pos[source_idx]
                self.sim_controller.set_right_position(pos=right_hand_pos)


    def joint_msg(self,hand,position,velocity,effort):
        # 初始化JointState消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        if hand == "left":
            joint_state_msg.name = []  # 关节名称
        elif hand == "right":
            joint_state_msg.name = []  # 关节名称
        joint_state_msg.position = position  # 关节位置（弧度）
        joint_state_msg.velocity = velocity  # 关节速度
        joint_state_msg.effort = effort  # 关节力矩
        return joint_state_msg

            
    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello ROS 2!'
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = LinkerHandPyBulletNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
