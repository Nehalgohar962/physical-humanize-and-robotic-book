import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import random

# This is a conceptual ROS 2 node that simulates high-level AI commands
# being translated into low-level motion commands for a humanoid.

class AIMotionControlNode(Node):
    def __init__(self):
        super().__init__('ai_motion_control_node')
        self.cmd_sub = self.create_subscription(
            String,
            'high_level_ai_commands',
            self.command_callback,
            10
        )
        self.joint_pub = self.create_publisher(Float32MultiArray, 'joint_position_commands', 10)
        self.get_logger().info('AI Motion Control Node started. Waiting for high-level commands.')

    def command_callback(self, msg):
        high_level_command = msg.data
        self.get_logger().info(f'Received AI command: "{high_level_command}"')

        low_level_commands = self.translate_ai_to_motion(high_level_command)
        
        joint_msg = Float32MultiArray()
        joint_msg.data = low_level_commands
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f'Translated to low-level joint commands: {low_level_commands}')

    def translate_ai_to_motion(self, command):
        """
        Conceptual function to translate high-level AI commands
        into a sequence of low-level joint position commands.
        """
        if "walk forward" in command.lower():
            # Simulate a simple walking motion (e.g., hip, knee, ankle joints)
            return [random.uniform(-0.1, 0.1), random.uniform(0.5, 0.8), random.uniform(-0.2, 0.2)]
        elif "raise arm" in command.lower():
            # Simulate raising an arm
            return [random.uniform(0.0, 1.5), random.uniform(0.0, 0.5), random.uniform(0.0, 0.0)]
        elif "balance" in command.lower():
            # Simulate slight joint adjustments for balance
            return [random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05)]
        else:
            self.get_logger().warn(f'Unknown AI command: "{command}"')
            return [0.0, 0.0, 0.0] # Default to no movement

def main(args=None):
    rclpy.init(args=args)
    node = AIMotionControlNode()
    
    # Simulate sending a high-level command to the node
    # In a real system, this would come from an LLM Commander node or similar
    simulated_command_publisher = node.create_publisher(String, 'high_level_ai_commands', 10)
    
    def publish_simulated_command():
        cmd_msg = String()
        cmds = ["walk forward", "raise arm", "balance", "walk forward"]
        cmd_msg.data = random.choice(cmds)
        simulated_command_publisher.publish(cmd_msg)
        node.get_logger().info(f'Simulating high-level command: "{cmd_msg.data}"')

    timer = node.create_timer(5.0, publish_simulated_command)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
