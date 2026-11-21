#!/usr/bin/env python3
"""
Go2 真机关节状态桥接节点。

订阅 unitree_go/msg/LowState (topic: lowstate)，将其中 motor_state[i].q
转换为 sensor_msgs/msg/JointState 发布到 /joint_states，供 robot_state_publisher
和 RViz 使用，从而在 RViz 中显示真机的实时关节角度。

电机索引与 URDF 关节名映射 (来自 unitree_ros2 example/include/common/motor_crc.h):
    0: FR_hip_joint      1: FR_thigh_joint     2: FR_calf_joint
    3: FL_hip_joint      4: FL_thigh_joint     5: FL_calf_joint
    6: RR_hip_joint      7: RR_thigh_joint     8: RR_calf_joint
    9: RL_hip_joint     10: RL_thigh_joint    11: RL_calf_joint
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

# 电机索引 -> URDF joint name
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


class Go2JointStatePublisher(Node):
    def __init__(self):
        super().__init__("go2_joint_state_publisher")

        # 默认订阅高频 lowstate；若需要低频可改 "lf/lowstate"
        self.declare_parameter("source_topic", "lowstate")
        source_topic = self.get_parameter("source_topic").value

        self.declare_parameter("publish_topic", "joint_states")
        publish_topic = self.get_parameter("publish_topic").value

        self.pub = self.create_publisher(JointState, publish_topic, 10)
        self.sub = self.create_subscription(
            LowState, source_topic, self.callback, 10)
        self.get_logger().info(
            f"Go2JointStatePublisher: '{source_topic}' -> '{publish_topic}'")

    def callback(self, msg: LowState):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(JOINT_NAMES)
        out.position = [msg.motor_state[i].q for i in range(12)]
        out.velocity = [msg.motor_state[i].dq for i in range(12)]
        out.effort = [msg.motor_state[i].tau_est for i in range(12)]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Go2JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
