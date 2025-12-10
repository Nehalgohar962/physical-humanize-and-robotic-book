import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

# This is a conceptual ROS 2 node for sending actuator commands.
# In a real system, this would interface with ros2_control or a robot's hardware interface.

class ActuatorCommanderNode(Node):
    def __init__(self):
        super().__init__('actuator_commander_node')
        # Publishers for conceptual joint controllers
        self.shoulder_pitch_pub = self.create_publisher(Float64, '/shoulder_pitch_controller/commands', 10)
        self.elbow_pitch_pub = self.create_publisher(Float64, '/elbow_pitch_controller/commands', 10)
        self.get_logger().info('Actuator Commander Node started.')

        # Simple sequence of commands
        self.timer = self.create_timer(2.0, self.send_commands_sequence)
        self.command_step = 0

    def send_commands_sequence(self):
        msg = Float64()

        if self.command_step == 0:
            self.get_logger().info('Commanding arm to neutral position (0.0, 0.0)...')
            msg.data = 0.0
            self.shoulder_pitch_pub.publish(msg)
            self.elbow_pitch_pub.publish(msg)
        elif self.command_step == 1:
            self.get_logger().info('Commanding arm to reach forward (0.5, 1.0)...')
            msg.data = 0.5
            self.shoulder_pitch_pub.publish(msg)
            msg.data = 1.0
            self.elbow_pitch_pub.publish(msg)
        elif self.command_step == 2:
            self.get_logger().info('Commanding arm to retract (-0.8, -0.5)...')
            msg.data = -0.8
            self.shoulder_pitch_pub.publish(msg)
            msg.data = -0.5
            self.elbow_pitch_pub.publish(msg)
        else:
            self.get_logger().info('Command sequence finished.')
            self.timer.cancel() # Stop the timer

        self.command_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
