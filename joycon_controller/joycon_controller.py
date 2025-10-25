# joycon_pitch_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from joycon_controller.joycon_ahrs_rumble import JoyConAHRSRumbler, RumbleJoyCon
from pyjoycon import get_R_id, JoyCon

class JoyConController(Node):
    def __init__(self):
        super().__init__('joycon_controller')
        # self.jc_r = JoyConAHRSRumbler("R", alpha=0.03)
        # # self.jc_l = JoyConAHRSRumbler("L", alpha=0.03)
        self.pub_pitch_r = self.create_publisher(Float64, '~/ref_pitch_r', 10)
        self.pub_pitch_l = self.create_publisher(Float64, '~/ref_pitch_l', 10)
        self.sub_rumble = self.create_subscription(Float64, '~/rumble', self.on_rumble, 10)
        # self.create_timer(0.01, self.on_timer)  # 100 Hz
        # self.jc_r.recenter_yaw()
        # # self.jc_l.recenter_yaw()
        joycon_id = get_R_id()
        self.jc_rumble_r = RumbleJoyCon(*joycon_id)

    def on_timer(self):
        pitch, roll, yaw = self.jc_r.get_euler_rad()  # (pitch, roll, yaw) に整形済みとするなら適宜変更
        msg = Float64()
        msg.data = yaw
        self.pub_pitch_r.publish(msg)

        # pitch, roll, yaw = self.jc_l.get_euler_rad()
        # msg = Float64()
        # msg.data = yaw
        # self.pub_pitch_l.publish(msg)

    def on_rumble(self, msg: Float64):
        amp = float(msg.data)
        self.jc_rumble_r.rumble_simple()
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
