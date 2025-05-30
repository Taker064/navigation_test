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
            self.change_coordinate_callback,
            10
        )
        self.subscription2 = self.create_subscription(
            String,
            "/result_nav",
            self.navresult_callback,
            10,
        )
        # 状態フラグ: 現在、初期位置に戻るタスクを実行中かどうか
        self.is_returning_to_initial = False
        self.get_logger().info("Room_Coordinate node started.")

        self.subscription
        self.subscription2
    
    def change_coordinate_callback(self,room):
        # 新しい外部からの目標指示なので、初期位置復帰フラグをリセット
        self.is_returning_to_initial = False 

        roomname = room.data
        msg = PoseStamped()

        
        msg.header.stamp = self.get_clock().now().to_msg() # 現在時刻
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
        elif roomname == "room2":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0  # クォータニオン (w=1は回転なし)
        elif roomname == "room3":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
        elif roomname == "room4":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
        elif roomname == "room5":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
        elif roomname == "room6":
            msg.pose.position.x = 2.0
            msg.pose.position.y = 7.0
            msg.pose.position.z = 0.0

            # 姿勢データの設定 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

        else:
            self.get_logger().info("Not found this room_name")
            return
        # メッセージの送信
        self.publisher.publish(msg)
        self.get_logger().info(f"send coordinate:{msg}")

    def navresult_callback(self,result):
        resultname = result.data
        if resultname == "SUCCEEDED":
            if not self.is_returning_to_initial:
                # 最初の目標への移動が成功した場合
                self.get_logger().info("Initial goal succeeded. Returning to initial position (0,0,0).")
                
                initial_pose_msg = PoseStamped()
                initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
                initial_pose_msg.header.frame_id = 'map'
                initial_pose_msg.pose.position.x = 0.0
                initial_pose_msg.pose.position.y = 0.0
                initial_pose_msg.pose.position.z = 0.0
                initial_pose_msg.pose.orientation.w = 1.0 # 回転なし
                
                self.publisher.publish(initial_pose_msg)
                self.is_returning_to_initial = True # 初期位置へ戻るタスク実行中フラグを立てる
            else:
                # 初期位置への復帰が成功した場合
                self.get_logger().info("Returned to initial position. Mission complete.")
                self.is_returning_to_initial = False #フラグリセット
        else:
            self.get_logger().info("goal_false")
            self.is_returning_to_initial = False # 失敗時もフラグリセット
            return

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


