#actionクライアント単体で初期位置に戻る場合
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
# GoalStatus をインポートしてステータスコードを比較できるようにする
from action_msgs.msg import GoalStatus

class NavigatorTest(Node):
    
    def __init__(self):
        super().__init__("goal_subscriber_navigator")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 初期位置を定義 (例として (0,0) 回転なし)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        # self.initial_pose.header.stamp は送信時に設定
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.position.z = 0.0
        self.initial_pose.pose.orientation.x = 0.0
        self.initial_pose.pose.orientation.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        
        # 状態フラグ: 現在、初期位置に戻るタスクを実行中かどうか
        self.is_returning_to_initial = False

        self.subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.external_goal_callback, # 外部からのゴール受信用コールバック名を変更
            10
        )
        self.get_logger().info("Goal subscriber navigator node has been started.")
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server is available.')
    
    def external_goal_callback(self, msg: PoseStamped):
        """
        外部トピック (/goal_pose) から新しいゴールを受け取ったときのコールバック関数
        """
        self.get_logger().info(f'Received new external goal: (x: {msg.pose.position.x:.2f}, y: {msg.pose.position.y:.2f})')
        self.is_returning_to_initial = False # 新しい外部ゴールなのでフラグをリセット
        self.send_goal_to_nav2(msg)

    def send_goal_to_nav2(self, target_pose_msg: PoseStamped):
        """
        指定されたPoseStampedメッセージをNav2アクションサーバーに送信する共通関数
        """
        # ヘッダーのタイムスタンプを現在時刻で更新
        target_pose_msg.header.stamp = self.get_clock().now().to_msg()

        goal_action_msg = NavigateToPose.Goal()
        goal_action_msg.pose = target_pose_msg

        self.get_logger().info('Sending goal request to Nav2...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_action_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected by Nav2 server.")
            self.is_returning_to_initial = False # 念のためリセット
            return
        
        self.get_logger().info("Goal was accepted by Nav2 server.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        action_result = future.result() # アクション実行結果のラッパーを取得
        status = action_result.status   # ゴールステータスを取得
        # result = action_result.result # NavigateToPose.Result 型のメッセージ

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
            
            if not self.is_returning_to_initial:
                # 最初の (外部からの) ゴールに成功した場合
                self.get_logger().info("External goal reached. Now returning to initial pose.")
                self.is_returning_to_initial = True # 初期位置に戻るフラグを立てる
                self.send_goal_to_nav2(self.initial_pose) # 初期位置へのゴールを送信
            else:
                # 初期位置に戻るゴールに成功した場合
                self.get_logger().info("Successfully returned to initial pose. Mission complete.")
                self.is_returning_to_initial = False # 次の外部ゴールに備えてフラグをリセット
        else:
            self.get_logger().info(f'Goal failed or was canceled. Status: {status}')
            # 失敗時やキャンセル時もフラグはリセットしておく
            self.is_returning_to_initial = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance remaining: {feedback.distance_remaining:.2f} m')
        
def main(args=None):
    rclpy.init(args=args)
    goal_subscriber_navigator = NavigatorTest()
    try:
        rclpy.spin(goal_subscriber_navigator)
    except KeyboardInterrupt:
        pass
    finally:
        goal_subscriber_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()