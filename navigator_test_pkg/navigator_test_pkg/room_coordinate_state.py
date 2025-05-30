# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped #nav2座標系用のやつ

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import State
from yasmin import Blackboard

class Room_Coordinate_State(State):

    def __init__(self,node: Node):
        super().__init__(outcomes=["True","False"])
        #nodeオブジェクトのインスタンスを作成
        self.node = node
        self.publisher = self.node.create_publisher(
            PoseStamped,
            "/goal_pose",  # 送信先のトピック名
            10,
        )


    def execute(self, blackboard: Blackboard) -> str:

        # 新しい外部からの目標指示なので、初期位置復帰フラグをリセット


        roomname = blackboard.roomdata
        msg = PoseStamped()

        
        msg.header.stamp = self.node.get_clock().now().to_msg() # 現在時刻
        msg.header.frame_id = 'map'  # 座標系のフレームID.絶対座標ならmap,相対座標ならbase_link
        if roomname == "room1":
            # 座標データの設定 
            msg.pose.position.x = 1.0
            msg.pose.position.y = 2.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0  # クォータニオン (w=1は回転なし)
            # メッセージの送信
            self.publisher.publish(msg)
            self.node.get_logger().info(f"send coordinate:{msg}")
            return "True"
        elif roomname == "room2":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0  # クォータニオン (w=1は回転なし)
            self.publisher.publish(msg)
            self.node.get_logger().info(f"send coordinate:{msg}")
            return "True"
        elif roomname == "room3":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.publisher.publish(msg)
            self.node.get_logger().info(f"send coordinate:{msg}")
            return "True"
        elif roomname == "room4":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.publisher.publish(msg)
            self.node.get_logger().info(f"send coordinate:{msg}")
            return "True"
        elif roomname == "room5":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.publisher.publish(msg)
            self.node.get_logger().info(f"send coordinate:{msg}")
            return "True"
        elif roomname == "room6":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.publisher.publish(msg)
            self.node.get_logger().info(f"send coordinate:{msg}")
            return "True"

        else:
            self.node.get_logger().info("Not found this room_name")
            return "False"


