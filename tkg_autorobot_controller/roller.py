import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import math
import numpy as np


class Roller(Node):
    def __init__(self):
        super().__init__('roller')

        self.subscription = self.create_subscription(Float64, '/roller', self.callback, 10)

        self.control = False
        self.control_sub = self.create_subscription(Bool, '/control', self.control_callback, 10)

        self.motor0_rpm = 0
        self.motor1_rpm = 0
        self.subscription0 = self.create_subscription(Float64, '/can_node/c620_0/rpm', self.motor0_callback, 10)
        self.subscription1 = self.create_subscription(Float64, '/can_node/c620_1/rpm', self.motor1_callback, 10)

        self.c620_0_pub = self.create_publisher(Float64, '/can_node/c620_0/target_current', 10)
        self.c620_1_pub = self.create_publisher(Float64, '/can_node/c620_1/target_current', 10)

        self.pid0 = PID(0.0028, 0, 0.0)
        self.pid1 = PID(0.0028, 0, 0.0)

        self.stamp = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.timer_callback)

    def output0(self, value):
        self.c620_0_pub.publish(Float64(data=float(value)))

    def output1(self, value):
        self.c620_1_pub.publish(Float64(data=float(value)))

    def callback(self, msg):
        self.pid0.target(msg.data)
        self.pid1.target(-msg.data)

    def motor0_callback(self, msg):
        self.motor0_rpm = msg.data

    def motor1_callback(self, msg):
        self.motor1_rpm = msg.data

    def control_callback(self, msg):
        self.control = msg.data

    def timer_callback(self):
        diff_time_nsec = (self.get_clock().now()-self.stamp).nanoseconds
        self.stamp = self.get_clock().now()

        # 非常停止中にrpmが得られない場合にrpmの値を0に戻す
        cycle_check_ok = (diff_time_nsec <= 20000000)
        if not cycle_check_ok:
            self.motor0_rpm = 0
            self.motor1_rpm = 0

        self.pid0.update(self.motor0_rpm)
        self.pid1.update(self.motor1_rpm)

        if self.control:
            self.output0(self.pid0.output())
            self.output1(self.pid1.output())
        else:
            self.output0(0)
            self.output1(0)


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._target = 0
        self._output = 0
        self.value_i = 0
        self._last_diff = 0

        # 摩擦パラメータ
        self.static_friction_torque = 0.325 # 静止摩擦トルク 発振しない程度に上げる
        self.stribeck_vel = 0.05 # Stribeck速度　摩擦の影響を大きく受ける範囲を表す
        self.torque_constant = 0.5

    def target(self, value):
        self._target = value
    
    def update(self, value):
        diff = self._target-value

        value_p = diff*self.kp
        self.value_i += diff*self.ki
        value_d = (self._last_diff-diff)*self.kd

        self._output = value_p + self.value_i + value_d

        target_angular_vel = self._target / 60 * 2 * math.pi
        current_angular_vel = value / 60 * 2 * math.pi
        # 静止摩擦力分を加算
        friction_torque =  (self.static_friction_torque * np.exp(- (current_angular_vel / self.stribeck_vel) ** 2)) * math.tanh(target_angular_vel / 0.001)
        self._output += (-friction_torque) / self.torque_constant

        self._last_diff = diff

        if self._output > 16.0:
            self._output = 16.0
        if self._output < -16.0:
            self._output = -16.0
        return self._output

    def output(self):
        return self._output

def main():
    rclpy.init()
    node = Roller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()