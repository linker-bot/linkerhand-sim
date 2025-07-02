#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy,rospkg
import pybullet as p
import pybullet_data
import time,os,sys
from std_msgs.msg import String,Header
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg
from utils.mapping import *
from utils.l20_sim_controller import L20SimController
from utils.l21_sim_controller import L21SimController
from utils.t24_sim_controller import T24SimController
from utils.l10_sim_controller import L10SimController
from utils.l7_sim_controller import L7SimController

'''
五指全部展开
rostopic pub /cb_left_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0,0], velocity: [], effort: []}" -r 10
五指根部弯曲
rostopic pub /cb_left_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [-1.05,1.66,1.66,1.66,1.66,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], velocity: [], effort: []}" -r 10
五指横摆
rostopic pub /cb_left_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [0.0,0.0,0.0,0.0,0.0,1.45,-0.26,-0.26,-0.26,-0.26,0,0,0,0,0,0,0,0,0,0], velocity: [], effort: []}" -r 10
单独大拇指侧摆
rostopic pub /cb_left_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.84,0,0,0,0,0,0,0,0,0], velocity: [], effort: []}" -r 10

五指中关节弯曲
rostopic pub /cb_right_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.32,1.36,1.36,1.36,1.36], velocity: [], effort: []}" -r 10
'''
#
#urdf_path_left = rospack.get_path('linker_hand_pybullet') + "/urdf/linker_hand_l20_8_left.urdf"
class LinkerHandPybullet:
    def __init__(self,hand=None):
        self.hand = hand
        if self.hand == "L20":
            self.left_hand_sim_position = [0] * 26 # urdf有26个关节，需要进行数据转换
            self.right_hand_sim_position = [0] * 26 # urdf有26个关节，需要进行数据转换
            self.l20_sim = L20SimController()
            #print(f"机器人关节总数: {self.left_hand_num_joints}")
            rospy.Subscriber("/cb_left_hand_control_cmd",JointState,self.l20_left_hand_cmd_callback,queue_size=10)
            rospy.Subscriber("/cb_right_hand_control_cmd",JointState,self.l20_right_hand_cmd_callback,queue_size=10)
            ColorMsg(msg=f"当前模拟环境为{self.hand}", color="green")
            self.l20_sim.showSim()
        elif self.hand == "L7":
            self.l7_sim = L7SimController()
            rospy.Subscriber("/cb_left_hand_control_cmd",JointState,self.l7_left_hand_cmd_callback,queue_size=10)
            rospy.Subscriber("/cb_right_hand_control_cmd",JointState,self.l7_right_hand_cmd_callback,queue_size=10)
            ColorMsg(msg=f"当前模拟环境为{self.hand}", color="green")
            self.l7_sim.showSim()
        elif self.hand == "L10":
            self.l10_sim = L10SimController()
            rospy.Subscriber("/cb_left_hand_control_cmd",JointState,self.l10_left_hand_cmd_callback,queue_size=10)
            rospy.Subscriber("/cb_right_hand_control_cmd",JointState,self.l10_right_hand_cmd_callback,queue_size=10)
            ColorMsg(msg=f"当前模拟环境为{self.hand}", color="green")
            self.l10_sim.showSim()
        elif self.hand == "L21":
            self.l21_sim = L21SimController()
            rospy.Subscriber("/cb_left_hand_control_cmd",JointState,self.l21_left_hand_cmd_callback,queue_size=10)
            rospy.Subscriber("/cb_right_hand_control_cmd",JointState,self.l21_right_hand_cmd_callback,queue_size=10)
            ColorMsg(msg=f"当前模拟环境为{self.hand}", color="green")
            self.l21_sim.run()
        elif self.hand == "T24":
            self.t24_sim = T24SimController()
            rospy.Subscriber("/cb_right_hand_control_cmd",JointState,self.t24_right_hand_cmd_callback,queue_size=10)
            ColorMsg(msg=f"当前模拟环境为{self.hand}", color="green")
            self.t24_sim.run()

    def l7_left_hand_cmd_callback(self, msg):
        left_hand_pos = [0] * 25
        cmd_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L7")
        '''
        thumb_joint0
        thumb_joint1
        thumb_joint2
        thumb_joint3
        thumb_joint4
        thumb_joint5
        index_joint0
        index_joint1
        index_joint2
        index_joint3
        index_joint4
        middle_joint0
        middle_joint1
        middle_joint2
        middle_joint3
        ring_joint0
        ring_joint1
        ring_joint2
        ring_joint3
        ring_joint4
        little_joint0
        little_joint1
        little_joint2
        little_joint3
        little_joint4
        '''
        
        left_hand_pos[0] = cmd_pos[6]
        left_hand_pos[1] = cmd_pos[1]
        left_hand_pos[2] = cmd_pos[0]
        left_hand_pos[3] = cmd_pos[0]*3
        left_hand_pos[4] = cmd_pos[0]*2
        left_hand_pos[5] = cmd_pos[0]*2
        left_hand_pos[7] = cmd_pos[2]
        left_hand_pos[8] = cmd_pos[2]
        left_hand_pos[9] = cmd_pos[2]
        left_hand_pos[11] = cmd_pos[3]
        left_hand_pos[12] = cmd_pos[3]
        left_hand_pos[13] = cmd_pos[3]
        #left_hand_pos[15] = cmd_pos[7]
        left_hand_pos[16] = cmd_pos[4]
        left_hand_pos[17] = cmd_pos[4]
        left_hand_pos[18] = cmd_pos[4]
        #left_hand_pos[20] = cmd_pos[8]
        left_hand_pos[21] = cmd_pos[5]
        left_hand_pos[22] = cmd_pos[5]
        left_hand_pos[23] = cmd_pos[5]
        self.l7_sim.set_left_position(pos=left_hand_pos)
        

    def l7_right_hand_cmd_callback(self, msg):
        pass

    def l10_left_hand_cmd_callback(self, msg):
        left_hand_pos = [0] * 24
        cmd_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L10")
        left_hand_pos[0] = cmd_pos[9]
        left_hand_pos[1] = cmd_pos[1]
        left_hand_pos[2] = cmd_pos[0]
        left_hand_pos[3] = cmd_pos[0]
        left_hand_pos[4] = cmd_pos[0]
        left_hand_pos[6] = cmd_pos[6]
        left_hand_pos[7] = cmd_pos[2]
        left_hand_pos[8] = cmd_pos[2]
        left_hand_pos[9] = cmd_pos[2]
        left_hand_pos[11] = cmd_pos[3]
        left_hand_pos[12] = cmd_pos[3]
        left_hand_pos[13] = cmd_pos[3]
        left_hand_pos[15] = cmd_pos[7]
        left_hand_pos[16] = cmd_pos[4]
        left_hand_pos[17] = cmd_pos[4]
        left_hand_pos[18] = cmd_pos[4]
        left_hand_pos[20] = cmd_pos[8]
        left_hand_pos[21] = cmd_pos[5]
        left_hand_pos[22] = cmd_pos[5]
        left_hand_pos[23] = cmd_pos[5]
        self.l10_sim.set_left_position(pos=left_hand_pos)
    def l10_right_hand_cmd_callback(self, msg):
        right_hand_pos = [0] * 24
        cmd_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L10")
        right_hand_pos[0] = cmd_pos[9]
        right_hand_pos[1] = cmd_pos[1]
        right_hand_pos[2] = cmd_pos[0]
        right_hand_pos[3] = cmd_pos[0]
        right_hand_pos[4] = cmd_pos[0]
        right_hand_pos[6] = cmd_pos[6]
        right_hand_pos[7] = cmd_pos[2]
        right_hand_pos[8] = cmd_pos[2]
        right_hand_pos[9] = cmd_pos[2]
        right_hand_pos[11] = cmd_pos[3]
        right_hand_pos[12] = cmd_pos[3]
        right_hand_pos[13] = cmd_pos[3]
        right_hand_pos[15] = cmd_pos[7]
        right_hand_pos[16] = cmd_pos[4]
        right_hand_pos[17] = cmd_pos[4]
        right_hand_pos[18] = cmd_pos[4]
        right_hand_pos[20] = cmd_pos[8]
        right_hand_pos[21] = cmd_pos[5]
        right_hand_pos[22] = cmd_pos[5]
        right_hand_pos[23] = cmd_pos[5]
        self.l10_sim.set_right_position(pos=right_hand_pos)


    # 左手回调
    def l20_left_hand_cmd_callback(self, msg):
        # 4,5,9,10,13,14,15,19,20,24,25
        cmd_pos = list(msg.position)
        # 五指根部弯曲
        self.left_hand_sim_position[0] = self.map_value(cmd_pos[0],to_min=-1.05,to_max=0.49, from_min=0, from_max=255)
        self.left_hand_sim_position[7] = self.map_value(cmd_pos[1],to_min=0.0,to_max=1.66)
        self.left_hand_sim_position[12] = self.map_value(cmd_pos[2],to_min=0.0,to_max=1.66)
        self.left_hand_sim_position[17] = self.map_value(cmd_pos[3],to_min=0.0,to_max=1.66)
        self.left_hand_sim_position[22] = self.map_value(cmd_pos[4],to_min=0.0,to_max=1.66)
        # 五指横摆
        self.left_hand_sim_position[1] = self.map_value(cmd_pos[5],to_min=0.0,to_max=1.45)
        self.left_hand_sim_position[6] = self.map_value(cmd_pos[6],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        self.left_hand_sim_position[11] = self.map_value(cmd_pos[7],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        self.left_hand_sim_position[16] = self.map_value(cmd_pos[8],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        self.left_hand_sim_position[21] = self.map_value(cmd_pos[9],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        # 单独大拇指侧摆
        self.left_hand_sim_position[2] = -self.map_value(cmd_pos[10],to_min=0.0,to_max=0.84)
        # 五指中关节弯曲
        self.left_hand_sim_position[3] = -self.map_value(cmd_pos[15],to_min=0.0,to_max=1.34)
        self.left_hand_sim_position[8] = self.map_value(cmd_pos[16],to_min=0.0,to_max=1.36)
        self.left_hand_sim_position[13] = self.map_value(cmd_pos[17],to_min=0.0,to_max=1.36)
        self.left_hand_sim_position[18] = self.map_value(cmd_pos[18],to_min=0.0,to_max=1.36)
        self.left_hand_sim_position[23] = self.map_value(cmd_pos[19],to_min=0.0,to_max=1.36)
        self.l20_sim.set_left_position(pos=self.left_hand_sim_position)
    
    # 右手回调
    def l20_right_hand_cmd_callback(self, msg):
        cmd_pos = list(msg.position)
        # 五指根部弯曲
        self.right_hand_sim_position[0] = -self.map_value(cmd_pos[0],to_min=-1.05,to_max=0.49, from_min=0, from_max=255)
        self.right_hand_sim_position[7] = self.map_value(cmd_pos[1],to_min=0.0,to_max=1.66)
        self.right_hand_sim_position[12] = self.map_value(cmd_pos[2],to_min=0.0,to_max=1.66)
        self.right_hand_sim_position[17] = self.map_value(cmd_pos[3],to_min=0.0,to_max=1.66)
        self.right_hand_sim_position[22] = self.map_value(cmd_pos[4],to_min=0.0,to_max=1.66)
        # 五指横摆
        self.right_hand_sim_position[1] = -self.map_value(cmd_pos[5],to_min=0.0,to_max=1.45)
        self.right_hand_sim_position[6] = -self.map_value(cmd_pos[6],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        self.right_hand_sim_position[11] = -self.map_value(cmd_pos[7],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        self.right_hand_sim_position[16] = -self.map_value(cmd_pos[8],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        self.right_hand_sim_position[21] = -self.map_value(cmd_pos[9],to_min=-0.26,to_max=0.26, from_min=255, from_max=0)
        # 单独大拇指侧摆
        self.right_hand_sim_position[2] = -self.map_value(cmd_pos[10],to_min=0.0,to_max=0.84)
        # 五指中关节弯曲
        self.right_hand_sim_position[3] = self.map_value(cmd_pos[15],to_min=0.0,to_max=1.34)
        self.right_hand_sim_position[8] = self.map_value(cmd_pos[16],to_min=0.0,to_max=1.36)
        self.right_hand_sim_position[13] = self.map_value(cmd_pos[17],to_min=0.0,to_max=1.36)
        self.right_hand_sim_position[18] = self.map_value(cmd_pos[18],to_min=0.0,to_max=1.36)
        self.right_hand_sim_position[23] = self.map_value(cmd_pos[19],to_min=0.0,to_max=1.36)
        self.l20_sim.set_right_position(pos=self.right_hand_sim_position)

    def l21_left_hand_cmd_callback(self, msg):
        left_hand_pos = [0.0] * 17
        cmd_pos = list(msg.position)
        mapping = {
                0: 6,  1: 1,   2: 21,   
                3: 7,  4: 2,  5: 22,
                6: 8, 7: 3,   8: 23,
                9: 9,  10: 4, 11: 24,
                12: 10, 13: 5, 
                14: 0,  15: 15, 16: 20
            }
        #[(-0.18, 0.18),(0, 1.57),(0, 1.57),(-0.18, 0.18),(0, 1.57),(0, 1.57),(-0.18, 0.18),(0, 1.57),(0, 1.57),(0, 0.18),(0, 1.57),(0, 1.57),(-0.6, 0.6),(0, 1.6),(0, 1),(0, 1.57),(0, 1.57)]
        left_hand_pos[0] = self.map_value(cmd_pos[6],to_min=-0.18,to_max=0.18)
        left_hand_pos[1] = self.map_value(cmd_pos[1],to_min=0,to_max=1.57)
        left_hand_pos[2] = self.map_value(cmd_pos[21],to_min=0,to_max=1.57)
        left_hand_pos[3] = self.map_value(cmd_pos[7],to_min=-0.18,to_max=0.18)
        left_hand_pos[4] = self.map_value(cmd_pos[2],to_min=0,to_max=1.57)
        left_hand_pos[5] = self.map_value(cmd_pos[22],to_min=0,to_max=1.57)
        left_hand_pos[6] = self.map_value(cmd_pos[8],to_min=-0.18,to_max=0.18)
        left_hand_pos[7] = self.map_value(cmd_pos[3],to_min=0,to_max=1.57)
        left_hand_pos[8] = self.map_value(cmd_pos[23],to_min=0,to_max=1.57)
        left_hand_pos[9] = self.map_value(cmd_pos[9],to_min=0,to_max=0.18)
        left_hand_pos[10] = self.map_value(cmd_pos[4],to_min=0,to_max=1.57)
        left_hand_pos[11] = self.map_value(cmd_pos[24],to_min=0,to_max=1.57)
        left_hand_pos[12] = self.map_value(cmd_pos[10],to_min=-0.6,to_max=0.6)
        left_hand_pos[13] = self.map_value(cmd_pos[5],to_min=0,to_max=1.6)
        left_hand_pos[14] = self.map_value(cmd_pos[0],to_min=0,to_max=1.0)
        left_hand_pos[15] = self.map_value(cmd_pos[15],to_min=0,to_max=1.57)
        left_hand_pos[16] = self.map_value(cmd_pos[20],to_min=0,to_max=1.57)
        self.l21_sim.set_left_position(pos=left_hand_pos)

    def l21_right_hand_cmd_callback(self, msg):
        right_hand_pos = [0.0] * 17
        cmd_pos = list(msg.position)
        mapping = {
                0: 6,  1: 1,   2: 21,   
                3: 7,  4: 2,  5: 22,
                6: 8, 7: 3,   8: 23,
                9: 9,  10: 4, 11: 24,
                12: 10, 13: 5, 
                14: 0,  15: 15, 16: 20
            }
        right_hand_pos[0] = self.map_value(cmd_pos[6],to_min=-0.18,to_max=0.18)
        right_hand_pos[1] = self.map_value(cmd_pos[1],to_min=0,to_max=1.57)
        right_hand_pos[2] = self.map_value(cmd_pos[21],to_min=0,to_max=1.57)
        right_hand_pos[3] = self.map_value(cmd_pos[7],to_min=-0.18,to_max=0.18)
        right_hand_pos[4] = self.map_value(cmd_pos[2],to_min=0,to_max=1.57)
        right_hand_pos[5] = self.map_value(cmd_pos[22],to_min=0,to_max=1.57)
        right_hand_pos[6] = self.map_value(cmd_pos[8],to_min=-0.18,to_max=0.18)
        right_hand_pos[7] = self.map_value(cmd_pos[3],to_min=0,to_max=1.57)
        right_hand_pos[8] = self.map_value(cmd_pos[23],to_min=0,to_max=1.57)
        right_hand_pos[9] = self.map_value(cmd_pos[9],to_min=0,to_max=0.18)
        right_hand_pos[10] = self.map_value(cmd_pos[4],to_min=0,to_max=1.57)
        right_hand_pos[11] = self.map_value(cmd_pos[24],to_min=0,to_max=1.57)
        right_hand_pos[12] = self.map_value(cmd_pos[10],to_min=-0.6,to_max=0.6)
        right_hand_pos[13] = self.map_value(cmd_pos[5],to_min=0,to_max=1.6)
        right_hand_pos[14] = self.map_value(cmd_pos[0],to_min=0,to_max=1.0)
        right_hand_pos[15] = self.map_value(cmd_pos[15],to_min=0,to_max=1.57)
        right_hand_pos[16] = self.map_value(cmd_pos[20],to_min=0,to_max=1.57)
        self.l21_sim.set_right_position(pos=right_hand_pos)

    def t24_right_hand_cmd_callback(self, msg):
        right_hand_pos = [0.0] * 26
        cmd_pos = list(msg.position)
        mapping = {
            0:10, 1:5, 2:0, 3:15, 4:20, 6:6, 7:1, 8:16, 9:21,  11:7, 12:2, 13:17, 14:22, 16:8, 17:3, 18:18, 19:23, 21:9, 22:4, 23:19, 24:24
        }
        right_hand_pos[0] = self.map_value(cmd_pos[10],to_min=-0.26,to_max=0.61)
        right_hand_pos[1] = self.map_value(cmd_pos[5],to_min=-1.43,to_max=0.0, from_max=255,from_min=0)
        right_hand_pos[2] = self.map_value(cmd_pos[0],to_min=0.0,to_max=0.89)
        right_hand_pos[3] = self.map_value(cmd_pos[15],to_min=-1.57,to_max=0, from_max=255,from_min=0)
        right_hand_pos[4] = self.map_value(cmd_pos[20],to_min=-1.57,to_max=0, from_max=255,from_min=0)
        right_hand_pos[6] = self.map_value(cmd_pos[6],to_min=-0.18,to_max=0)
        right_hand_pos[7] = self.map_value(cmd_pos[1],to_min=0,to_max=1.57)
        right_hand_pos[8] = self.map_value(cmd_pos[16],to_min=0,to_max=1.57)
        right_hand_pos[9] = self.map_value(cmd_pos[21],to_min=0,to_max=1.57)
        right_hand_pos[11] = self.map_value(cmd_pos[7],to_min=0,to_max=1.57)
        right_hand_pos[12] = self.map_value(cmd_pos[2],to_min=0,to_max=1.57)
        right_hand_pos[13] = self.map_value(cmd_pos[17],to_min=0,to_max=1.57)
        right_hand_pos[14] = self.map_value(cmd_pos[22],to_min=0,to_max=1.57)
        right_hand_pos[16] = self.map_value(cmd_pos[8],to_min=0,to_max=0.18)
        right_hand_pos[17] = self.map_value(cmd_pos[3],to_min=0,to_max=1.57)
        right_hand_pos[18] = self.map_value(cmd_pos[18],to_min=0,to_max=1.57)
        right_hand_pos[19] = self.map_value(cmd_pos[23],to_min=0,to_max=1.57)
        right_hand_pos[21] = self.map_value(cmd_pos[9],to_min=0,to_max=1.57)
        right_hand_pos[22] = self.map_value(cmd_pos[4],to_min=0,to_max=0.18)
        right_hand_pos[23] = self.map_value(cmd_pos[19],to_min=0,to_max=1.57)
        right_hand_pos[24] = self.map_value(cmd_pos[24],to_min=0,to_max=1.57)
        self.t24_sim.set_right_position(pos=right_hand_pos)

    def map_value(self,value, to_min, to_max, from_min=255, from_max=0):
        """
        将一个范围内的值映射到另一个范围，支持输入范围反向（例如 255 对应最小值，0 对应最大值）。

        参数：
        - value: 需要映射的值
        - from_min: 原始范围的最小值
        - from_max: 原始范围的最大值
        - to_min: 目标范围的最小值
        - to_max: 目标范围的最大值

        返回：
        - 映射后的值
        """
        # 检查原始范围是否有效
        if from_min == from_max:
            raise ValueError("原始范围的最小值和最大值不能相等")
        
        # 反转范围处理：如果 from_min > from_max，则调整计算顺序
        if from_min > from_max:
            scaled_value = (from_min - value) / (from_min - from_max)  # 归一化到 [0, 1]
        else:
            scaled_value = (value - from_min) / (from_max - from_min)  # 正常归一化到 [0, 1]

        # 映射到目标范围
        mapped_value = to_min + scaled_value * (to_max - to_min)
        return mapped_value

if __name__ == "__main__":
    # rosrun linker_hand_pybullet linker_hand_pybullet.py _hand_type:=L10
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("linker_hand_pybullet", anonymous=True)
    # 获取参数
    hand = rospy.get_param('~hand_type', default="L10")  # 默认获取全局参数
    rospy.loginfo(f"hand parameter: {hand}")
    if hand == None:
        rospy.loginfo(f"hand parameter: {hand}")
        exit()
    lp = LinkerHandPybullet(hand=hand)


