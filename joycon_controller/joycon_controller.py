import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pyjoycon import JoyCon, get_R_id, get_L_id, get_gyro_x

class JoyConController(Node):
    def __init__(self):
        super().__init__('joycon_controller')
        joycon_r_id = get_R_id()
        joycon_l_id = get_L_id()
        self.joycon_r = JoyCon(*joycon_r_id)
        self.joycon_l = JoyCon(*joycon_l_id)

        self.ref_theta_r = self.create_publisher(Float64, '~/ref_theta_r', 10)
        self.ref_theta_l = self.create_publisher(Float64, '~/ref_theta_l', 10)
        self.ref_pitch = self.create_publisher(Float64, '~/ref_pitch', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        status_r = self.joycon_r.get_status()
        status_l = self.joycon_l.get_status()
        print(status_l)
        gyro_r = status_r['gyro']
        gyro_l = status_l['gyro']

        ref_r = Float64()
        ref_l = Float64()

        # 右スティックジャイロで右腕上下
        ref_r.data = -gyro_r['y']
        
        self.ref_theta_r.publish(ref_r)

        # 左スティックジャイロで左腕上下
        # ref_l.data = -gyro_l['y']
        # self.ref_theta_l.publish(ref_l)

def main(args=None):
    rclpy.init(args=args)
    node = JoyConController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()