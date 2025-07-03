import time
import pybullet as p
import pybullet_data
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from .mapping import arc_to_range_left, arc_to_range_right


class L10SimController:
    def __init__(self,urdf_path_left=None, urdf_path_right=None):
        # self.left_hand_state_pub = rospy.Publisher("/cb_left_hand_state_sim",JointState,queue_size=10)
        # self.right_hand_state_pub = rospy.Publisher("/cb_right_hand_state_sim",JointState,queue_size=10)
        # rospack = rospkg.RosPack()
        # urdf_path_left = rospack.get_path('linker_hand_pybullet') + "/urdf/l10/left/linkerhand_l10_left.urdf"
        # urdf_path_right = rospack.get_path('linker_hand_pybullet') + "/urdf/l10/right/linkerhand_l10_right.urdf"
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
        self.left_position = [0] * 25
        self.right_position = [0] * 25
    def set_left_position(self, pos):
        self.left_position = pos
    def set_right_position(self, pos):
        self.right_position = pos
    def showSim(self):
        while True:
            p.stepSimulation()
            time.sleep(self.time_step)
            tmp_left = {
                "position":[0.0] * 10,
                "velocity":[0.0] * 10,
                "effort":[0.0] * 10
            }
            tmp_right = {
                "position":[0.0] * 10,
                "velocity":[0.0] * 10,
                "effort":[0.0] * 10
            }
            
            m = [2,1,7,11,16,21,6,15,20,0]
            for index in range(len(m)):
                i = m[index]
                tmp_left["position"][index] = self.left_position[i]
                tmp_right["position"][index] = self.right_position[i]

            self.set_joint(self.left_hand_id,self.left_position)
            self.left_hand_current_position = tmp_left["position"]

            self.set_joint(self.right_hand_id,self.right_position)
            self.right_hand_current_position = tmp_right["position"]
    def set_joint(self,hand_id, pos):
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