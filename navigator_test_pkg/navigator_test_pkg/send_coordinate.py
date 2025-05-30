import rclpy #ros2_pythonでは必須す
from rclpy.node import Node #ros2_pythonでは必須
from geometry_msgs.msg import PoseStamped #nav2座標系用のやつ

class Send_Coordinate(Node):
    def __init__(self):
        super().__init__('periodic_pose_publisher_node')



        # Publisherの作成。
        self.publisher_ = self.create_publisher(
            PoseStamped,
            "/goal_pose",  # 送信先のトピック名
            10
        )

        timer_period = 10.0  # 送信周期（秒）
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0  # 送信するデータを変化させるためのカウンター（デモ用）

        self.get_logger().info('Periodic Pose Publisher node has been started.')
        self.get_logger().info(f'Publishing to /getmetorypose every {timer_period} seconds.')

    def timer_callback(self):
        msg = PoseStamped()

        # ヘッダー情報の設定
        msg.header.stamp = self.get_clock().now().to_msg() # 現在時刻
        msg.header.frame_id = 'map'  # 座標系のフレームID 変更.mapだと絶対座標,base_linkだと相対座標になる。

        # 座標データの設定 (単位はmなので注意)
        msg.pose.position.x = 1.0 + float(self.counter)
        msg.pose.position.y = 2.0
        msg.pose.position.z = 0.0

        # 姿勢データの設定 
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0  # クォータニオン (w=1は回転なし)

        # メッセージの送信
        self.publisher_.publish(msg)

        # ログ出力
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    periodic_pose_publisher = Send_Coordinate()
    try:
        rclpy.spin(periodic_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        periodic_pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
