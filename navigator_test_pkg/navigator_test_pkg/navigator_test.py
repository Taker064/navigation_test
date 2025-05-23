import rclpy #ros2_pythonでは必須
from rclpy.action import ActionClient #action通信のClient側で呼び出す
from rclpy.node import Node #ros2_pythonでは必須
from geometry_msgs.msg import PoseStamped #nav2座標系用のやつ
from nav2_msgs.action import NavigateToPose #Actionファイルのインポート

class NavigatorTest(Node):
    
    def __init__(self):
        super().__init__("goal_subscriber_navigator")
        #クライアントの初期化
        self._action_client = ActionClient(self,NavigateToPose,"navigate_to_pose") #第2引数がサーバー側のノード名,第3引数アクション名(どちらもサーバ側で設定したものを持ってくる)

        #subscriberの設定.PoseStamped型の/goal_poseというTopicを受け取る
        self.subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            10
        )
        self.subscription #宣言されていない変数関連で警告を吐かれるのを防ぐ処理
        self.get_logger().info("Goal subscriber navigator node has been started.")
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server() # アクションサーバーが起動するまで待機
        self.get_logger().info('Nav2 action server is available.')
    
    def goal_pose_callback(self, msg:PoseStamped):
        """
        PoseStampedトピックを受け取ったときのコールバック関数
        """
        self.get_logger().info(f'Received new goal: (x: {msg.pose.position.x:.2f}, y: {msg.pose.position.y:.2f})')

        #actionファイルから呼び出し、そのまま座標を代入
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.get_logger().info('Sending goal request...')
        #ゴールの定義をし、非同期で送信する。戻り地はFutureオブジェクト
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feed_backcallback #フィードバックコールバックの登録
        )
        #ゴールが受け釣れられたかどうかのコールバック関数
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self,future):
        """
        アクションサーバがゴールを受けたかどうかの応答するようのコールバック関数 futureにはself._action_client.send_goal_asyncの返り値のFUTUREメゾットが入っている。
        """
        goal_handle = future.result() #結果情報の取得
        if not goal_handle.accepted:
            self.get_logger().info("Goalが拒否されました")
            return
        
        self.get_logger().info("Goalが承認されました")

        # ゴールの最終結果を取得するためのFutureを取得
        self._get_result_future = goal_handle.get_result_async()
        # 最終結果を処理するコールバックを登録
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        アクションの最終結果を処理するコールバック関す
        """
        result = future.result().result # resultの中身は NavigateToPose.Result 型 (例: nav2_msgs/action/NavigateToPose.action を参照)
        self.get_logger().info(f"Result:{result}")
        status = future.result().status
        if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal成功")
        else:
            self.get_logger().info(f'Goal失敗 ステータスは: {status}')

    def feedback_callback(self, feedback_msg):
        """
        フィードバック受診時のコールバック関数
        """
        feedback = feedback_msg.feedback
        # feedbackの中身は NavigateToPose.Feedback 型
        # 例: 現在位置、目的地までの距離など
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