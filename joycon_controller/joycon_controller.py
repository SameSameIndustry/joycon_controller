# joycon_pitch_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from joycon_controller.joycon_ahrs_rumble import JoyConAHRSRumbler
from pyjoycon import get_R_id, JoyCon

class JoyConController(Node):
    def __init__(self):
        super().__init__('joycon_controller')
        self.jc_r = JoyConAHRSRumbler("R", alpha=0.03, calibration_samples=300)
        self.jc_l = JoyConAHRSRumbler("L", alpha=0.03, calibration_samples=300)
        self.pub_theta_r = self.create_publisher(Float64, '~/ref_theta_r', 10)
        self.pub_theta_l = self.create_publisher(Float64, '~/ref_theta_l', 10)
        self.pub_pitch = self.create_publisher(Float64, '~/ref_pitch', 10)
        self.sub_rumble = self.create_subscription(Empty, '~/rumble', self.on_rumble, 10)
        self.create_timer(0.01, self.on_timer)  # 100 Hz
        self.jc_r.recenter_yaw()
        self.jc_l.recenter_yaw()

    def on_timer(self):
        pitch, roll, yaw = self.jc_r.get_euler_rad()  # (pitch, roll, yaw) に整形済みとするなら適宜変更
        msg_theta_r = Float64()
        msg_theta_r.data = yaw
        self.pub_theta_r.publish(msg_theta_r)
        msg_pitch = Float64()
        msg_pitch.data = pitch
        self.pub_pitch.publish(msg_pitch)

        pitch, roll, yaw = self.jc_l.get_euler_rad()
        msg_theta_l = Float64()
        msg_theta_l.data = yaw
        self.pub_theta_l.publish(msg_theta_l)

        self.jc_r.check_reset_yaw()
        self.jc_l.check_reset_yaw()

    def on_rumble(self, msg: Empty):
        self.jc_r.rumble_simple()
        self.jc_l.rumble_simple()
        # self.jc_r.rumble(amplitude=abs(amp), side="R" if amp >= 0 else "L", duration_ms=120)
        # self.jc_l.rumble(amplitude=abs(amp), side="L" if amp >= 0 else "L", duration_ms=120)

def main():
    rclpy.init()
    node = JoyConController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
