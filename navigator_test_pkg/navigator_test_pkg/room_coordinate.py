import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped #nav2座標系用のやつ

class Room_Coordinate(Node):
    def __init__(self):
        super().__init__("room_coordinater")

        self.publisher = self.create_publisher(
            PoseStamped,
            "/goal_pose",  # 送信先のトピック名
            10,
        )

        self.subscription = self.create_subscription(
            String,
            '/qr_code_data',
            10,
            self.change_coordinate_callback,
        )
        self.subscription
    
    def change_coordinate_callback(self,room):
        roomname = room.data
        msg = PoseStamped()

        
        msg.header.stamp = self.get_clock().now().to_msg() # 現在時刻
        msg.header.frame_id = 'map'  # 座標系のフレームID 変更
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
        elif roomname == "room2":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 8.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0  # クォータニオン (w=1は回転なし)
        else:
            self.get_logger().info("Not found this room_name")
            return
        # メッセージの送信
        self.publisher.publish(msg)
        self.get_logger().info(f"send coordinate:{msg}")

def main(args = None):
    rclpy.init(args=args)
    periodic_pose_publisher = Room_Coordinate()
    try:
        rclpy.spin(periodic_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        periodic_pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


