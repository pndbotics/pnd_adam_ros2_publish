import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # 修改导入的消息类型
import numpy as np

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')        
        self.publisher = self.create_publisher(
            JointState,  # 修改消息类型
            'joint_states', 
            10)
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz
        self.get_logger().info('发布者已启动，正在向/joint_states主题发送关节状态数据...')  # 更新日志信息
        self.counter = 0
        # 定义关节名称，与retarget.cpp中的joint_name_publisher_保持一致
        self.joint_names = [
            "dof_pos/waistRoll", "dof_pos/waistPitch", "dof_pos/waistYaw",
            "dof_pos/shoulderPitch_Left", "dof_pos/shoulderRoll_Left", "dof_pos/shoulderYaw_Left",
            "dof_pos/elbow_Left", "dof_pos/wristYaw_Left", "dof_pos/wristPitch_Left",
            "dof_pos/wristRoll_Left", "dof_pos/shoulderPitch_Right", "dof_pos/shoulderRoll_Right",
            "dof_pos/shoulderYaw_Right", "dof_pos/elbow_Right", "dof_pos/wristYaw_Right",
            "dof_pos/wristPitch_Right", "dof_pos/wristRoll_Right", "root_pos/z",
            "dof_pos/L_pinky_MCP_joint", "dof_pos/L_pinky_DIP_joint", "dof_pos/L_ring_MCP_joint",
            "dof_pos/L_ring_DIP_joint", "dof_pos/L_middle_MCP_joint", "dof_pos/L_middle_DIP_joint",
            "dof_pos/L_index_MCP_joint", "dof_pos/L_index_DIP_joint", "dof_pos/L_thumb_proximal",
            "dof_pos/L_thumb_PIP_joint", "dof_pos/L_thumb_MCP_joint1", "dof_pos/R_pinky_MCP_joint",
            "dof_pos/R_pinky_DIP_joint", "dof_pos/R_ring_MCP_joint", "dof_pos/R_ring_DIP_joint",
            "dof_pos/R_middle_MCP_joint", "dof_pos/R_middle_DIP_joint", "dof_pos/R_index_MCP_joint",
            "dof_pos/R_index_DIP_joint", "dof_pos/R_thumb_proximal", "dof_pos/R_thumb_PIP_joint",
            "dof_pos/R_thumb_MCP_joint1"
        ]

    def timer_callback(self):
        msg = JointState()  # 创建JointState消息
        msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        msg.name = self.joint_names  # 设置关节名称
        
        # 初始化position数组
        position_array = np.zeros(len(self.joint_names), dtype=np.float64)
        
        position_array[:3] = 0.0        # 腰部姿态  复合角范围建议从小到大尝试
        position_array[17] = 1.0        # base高度 范围[0.6m 1.0m]，站立时为1.0m，调整为0.6m时会有下蹲动作
        
        position_array[3:10] = 0.0      # 左臂关节 根据实际情况调整，单位为弧度
        position_array[10:17] = 0.0     # 右臂关节 根据实际情况调整，单位为弧度
        
        # 兼容动捕服数据，每只手有11个自由度，但是实际每只手有6个关节，赋值时按照下面所示的方式即可，不需要拆分11个自由度
        # 手指范围[0 pi]，0.0表示手指完全伸直，pi表示手指完全弯曲
        hand_control_left = np.zeros(6, dtype=np.float64)
        hand_control_left[0] = 0.0
        hand_control_left[1] = 0.0
        hand_control_left[2] = 0.0
        hand_control_left[3] = 0.0
        hand_control_left[4] = 0.0
        hand_control_left[5] = 0.0
        hand_control_right = np.zeros(6, dtype=np.float64)
        hand_control_right[0] = 0.0
        hand_control_right[1] = 0.0
        hand_control_right[2] = 0.0
        hand_control_right[3] = np.pi/4.0*np.sin(2*np.pi*self.counter/100.0/3.0) + np.pi/4.0
        hand_control_right[4] = 0.0
        hand_control_right[5] = 0.0
        
        position_array[18:20] = [0.0, hand_control_left[0]]   # 左手小拇指
        position_array[20:22] = [0.0, hand_control_left[1]]   # 左手无名指
        position_array[22:24] = [0.0, hand_control_left[2]]   # 左手中指
        position_array[24:26] = [0.0, hand_control_left[3]]   # 左手食指
        position_array[26:28] = [0.0, hand_control_left[4]]   # 左手拇指1
        position_array[28] = hand_control_left[5]             # 左手拇指2
        
        position_array[29:31] = [0.0, hand_control_right[0]]   # 右手小拇指
        position_array[31:33] = [0.0, hand_control_right[1]]   # 右手无名指
        position_array[33:35] = [0.0, hand_control_right[2]]   # 右手中指
        position_array[35:37] = [0.0, hand_control_right[3]]   # 右手食指
        position_array[37:39] = [0.0, hand_control_right[4]]   # 右手拇指1
        position_array[39] = hand_control_right[5]             # 右手拇指2

        # 测试用，根据需要调整
        position_array[0] = 0.2*np.sin(2*np.pi*self.counter/100.0/5.0) # 测试腰部
        position_array[1] = 0.2*np.sin(2*np.pi*self.counter/100.0/6.0) # 测试腰部
        position_array[2] = 0.2*np.sin(2*np.pi*self.counter/100.0/7.0) # 测试腰部
        position_array[7] = 0.5*np.sin(2*np.pi*self.counter/100.0) # 测试左胳膊小臂
        position_array[14] = 0.5*np.sin(2*np.pi*self.counter/100.0) # 测试右胳膊小臂
        position_array[17] = 1.0 - (0.1*np.sin(2*np.pi*self.counter/100.0/5.0)+0.1) # 测试base高度
        
        # 将numpy数组转换为list类型
        msg.position = position_array.tolist()
        
        msg.velocity = [0.0] * len(self.joint_names)  # 设置速度为零
        msg.effort = [0.0] * len(self.joint_names)  # 设置力矩为零
        
        self.publisher.publish(msg)
        self.get_logger().info(f'发布关节状态数据 #{self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = JointStatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()