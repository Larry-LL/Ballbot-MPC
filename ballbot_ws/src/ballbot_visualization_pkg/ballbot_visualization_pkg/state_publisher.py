import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_states)

        # Initialize translation and rotation positions
        self.positions = {'x': 0.0, 'y': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.speeds = {'x': 0.1, 'y': 0.1, 'pitch': 0.05, 'roll': 0.05}

    def publish_states(self):
        # Update translation
        self.positions['x'] += self.speeds['x'] * 0.01
        self.positions['y'] += self.speeds['y'] * 0.01

        # Update cylinder pitch and roll
        self.positions['pitch'] += 0  #self.speeds['pitch'] * 0.01
        self.positions['roll'] += 0 #self.speeds['roll'] * 0.01

        # Limit pitch and roll to realistic ranges (-pi to pi)
        self.positions['pitch'] = self.positions['pitch'] % (2 * 3.14159)
        self.positions['roll'] = self.positions['roll'] % (2 * 3.14159)

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'world_to_sphere_x',
            'x_to_y',
            'sphere_to_cylinder_pitch',
            'pitch_to_cylinder_roll'
        ]
        joint_state.position = [
            self.positions['x'],
            self.positions['y'],
            self.positions['pitch'],
            self.positions['roll']
        ]
        self.joint_state_publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state_publisher...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()





# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# class StatePublisher(Node):
#     def __init__(self):
#         super().__init__('state_publisher')
#         self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
#         self.timer = self.create_timer(0.01, self.publish_states)


#         # Initialize tilt angles
#         self.tilt_angles = {'roll': 0.0, 'pitch': 0.0}

#     def publish_states(self):
#         self.tilt_angles['roll'] += 0.00
#         self.tilt_angles['pitch'] += 0.00

#         joint_state = JointState()
#         joint_state.header.stamp = self.get_clock().now().to_msg()
#         joint_state.name = ['base_to_intermediate_x', 'intermediate_to_cylinder_y']
#         joint_state.position = [self.tilt_angles['roll'], self.tilt_angles['pitch']]
#         self.joint_state_publisher.publish(joint_state)

# def main(args=None):
#     rclpy.init(args=args)
#     node = StatePublisher()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down state_publisher...')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

