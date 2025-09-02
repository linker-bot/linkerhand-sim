import time
import pybullet as p
import pybullet_data
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from .mapping import arc_to_range_left, arc_to_range_right


class L6SimController:
    def __init__(self,urdf_path_left=None, urdf_path_right=None):
        self.left_hand_current_position = []
        self.right_hand_current_position = []
        # 连接到仿真
        physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # 加载 URDF 左手
        self.left_hand_id = p.loadURDF(urdf_path_left, basePosition=[0, -0.1, 0.1], useFixedBase=True)
        self.right_hand_id = p.loadURDF(urdf_path_right, basePosition=[0, 0.1, 0.1], useFixedBase=True)
        # 加载地面
        plane_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[10, 10, 0.1])
        plane_id = p.createMultiBody(0, plane_collision_shape)
        p.setPhysicsEngineParameter(enableFileCaching=0)
        # 重力
        p.setGravity(0, 0, -9.81)
        self.time_step = 1.0 / 240.0
        p.setTimeStep(self.time_step)
        # 获取关节总数和信息
        self.left_hand_num_joints = p.getNumJoints(self.left_hand_id)
        self.right_hand_num_joints = p.getNumJoints(self.right_hand_id)
        self.left_position = [0] * 6
        self.right_position = [0] * 6
    def set_left_position(self, pos):
        self.left_position = pos
    def set_right_position(self, pos):
        self.right_position = pos
    def showSim(self):
        while True:
            p.stepSimulation()
            time.sleep(self.time_step)
            tmp_left = {
                "position":[0.0] * 6,
                "velocity":[0.0] * 6,
                "effort":[0.0] * 6
            }
            tmp_right = {
                "position":[0.0] * 6,
                "velocity":[0.0] * 6,
                "effort":[0.0] * 6
            }

            #m = [2,1,7,11,16,21,0]
            m = [1,0,2,3,4,5]
            for index in range(len(m)):
                i = m[index]
                tmp_left["position"][index] = self.left_position[i]
                tmp_right["position"][index] = self.right_position[i]
            if len(self.left_position) > 0:
                self.set_joint(self.left_hand_id,self.left_position)
                self.left_hand_current_position = tmp_left["position"]
            if len(self.right_position) > 0:
                self.set_joint(self.right_hand_id,self.right_position)
                self.right_hand_current_position = tmp_right["position"]
    def set_joint(self,hand_id, pos):
        pos = pos[:11]
        for index, item in enumerate(pos):
            p.setJointMotorControl2(
                bodyUniqueId=hand_id,           # 机器人ID
                jointIndex=index,          # 关节索引
                controlMode=p.POSITION_CONTROL,  # 控制模式：位置控制
                targetPosition=item,  # 目标位置
                force=500                        # 最大力矩限制
            )

    def joint_msg(self,hand,position,velocity,effort):
        # 初始化JointState消息
        joint_state_msg = JointState()
        if hand == "left":
            joint_state_msg.name = []  # 关节名称
        elif hand == "right":
            joint_state_msg.name = []  # 关节名称
        joint_state_msg.position = position  # 关节位置（弧度）
        joint_state_msg.velocity = velocity  # 关节速度
        joint_state_msg.effort = effort  # 关节力矩
        return joint_state_msg
    