#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Vector3

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        
        # 各トピックのサブスクライバを作成（キューサイズは10）
        self.create_subscription(Int64, '/can_node/gm6020_1/target_volt', self.int_callback, 10)
        self.create_subscription(Vector3, '/current_turret_pose', self.float_callback, 10)
        
        # 最新の受信値を保持する変数
        self.last_int = None
        self.last_float = None
        
        # ログを書き出すファイルをオープン
        self.log_file = open('output.log', 'w')
        self.log_file.write("time,target_volt,yaw")
        
        # 0.01秒（10ms）ごとにタイマーコールバックを呼ぶ
        self.timer = self.create_timer(0.01, self.timer_callback)

    def int_callback(self, msg: Int64):
        # Int64トピックの値を更新
        self.last_int = msg.data

    def float_callback(self, msg: Vector3):
        # Float64トピックの値を更新
        self.last_float = msg.z

    def timer_callback(self):
        # 現在時刻（ROSの内部時刻）を取得
        now = self.get_clock().now()
        # 秒とナノ秒に分解してフォーマット
        sec = now.seconds_nanoseconds()[0]
        nsec = now.seconds_nanoseconds()[1]
        timestamp = f"{sec}.{nsec:09d}"
        
        # ログ文字列の作成（各値が未受信の場合はNoneと表示）
        log_line = f"{timestamp}, {self.last_int}, {self.last_float}\n"
        
        # ファイルに追記してフラッシュ
        self.log_file.write(log_line)
        self.log_file.flush()

    def destroy_node(self):
        # ノード破棄時にファイルをクローズ
        self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
