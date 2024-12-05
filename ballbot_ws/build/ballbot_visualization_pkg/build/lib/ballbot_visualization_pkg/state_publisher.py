import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
# from Full_mpc import x_pos, y_pos, pitch_angles, roll_angles  # Import state data
# ballbot_visualization_pkg.Full_mpc import x_pos, y_pos, pitch_angles, roll_angles

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.4, self.publish_states)
        self.obstacle_timer = self.create_timer(0.4, self.publish_obstacle_marker)  # Fast rate (0.1 seconds)


        # Initialize translation and rotation positions
        self.x_positions = [0, 2.9536227731502885e-20, -0.10988999132466687, -0.17283700015457465, -0.14632126522834946, -0.07894392406786631, 0.04986091070602829, 0.22068548165543297, 0.39286967713895915, 0.5683275702628672, 0.7339524051724838, 0.8606600113330396, 0.9037226735302745, 1.013450834137602, 1.0758833020434995, 1.1182613061510325, 1.1449831167789413, 1.1560205236341161, 1.169624737197094, 1.1771442362888138, 1.1744916312839557, 1.1725469214172766, 1.179133150429813, 1.1854257428837414, 1.1901708834192748, 1.1935135825785106, 1.1958188624717716, 1.1973844850587916, 1.1984348546993744]

        self.y_positions = [0, -4.5884025560176105e-20, -0.21300599916791368, -0.26335400231626055, -0.1622843749669759, -0.11993675438000118, 0.16533895174295593, 0.45167901722858533, 0.7065955359980916, 0.8247130458710864, 0.9311171988676491, 1.0296706326691798, 1.1843083667975585, 1.3200496858486597, 1.4810517798252711, 1.61143403503135, 1.7150363144420622, 1.7978334471141533, 1.8750837029948215, 1.925033973033885, 1.9685644800132962, 1.982832719310001, 1.9848971179555999, 1.985331515960976, 1.9860025711783702, 1.9870564119527163, 1.9883458748118714, 1.9897239005064657, 1.9910856615688493]
        self.pitch_angles = [0, -1.935015737501338e-20, 0.038924657930140494, 0.048125238231499834, 0.036599629126399524, 0.044271285678274805, 0.014110092338692645, -0.008420789819903449, -0.02271510886727546, -0.01350313302589284, -0.006169760994557971, 0.0002092633210859537, -0.004751414977881071, -0.006221947447541692, -0.013149023830181355, -0.015581655726495475, -0.015447757379414685, -0.01426949994334188, -0.0148104728507585, -0.0128883085386046, -0.012413185248559772, -0.008871870767361076, -0.0052967665801542455, -0.00299327391425984, -0.0016700528943668373, -0.0009463125659867129, -0.0005610747572752061, -0.00035943099601353705, -0.0002540735993644234]
        self.roll_angles = [0, 1.9735333496989298e-20, 0.02008126690782259, 0.03158418605564509, 0.030300432416670613, 0.027145513168259777, 0.018138741468275026, 0.006262271878422665, -0.0026541730469110117, -0.011060992668729558, -0.018140483930159632, -0.02006621657189608, -0.009922916456909852, -0.015503834888177696, -0.014198586206157464, -0.011972332623482714, -0.009397524276315244, -0.006075017124387041, -0.0048839279376894215, -0.003655680056647512, -0.0014326464776562256, 1.4314672034064377e-05, -0.00035110706884342496, -0.0006603373664768458, -0.0007489003975273702, -0.0006980007339293249, -0.0005900098985807302, -0.0004703362226726749, -0.00036088282472754416]
        self.index = 0

        #obstacle related 
        self.frequency = 0.5
        self.amplitude = 1
        self.t = 0

    def publish_states(self):
        # Update translation
        if self.index < len(self.x_positions):

            x = float(self.x_positions[self.index])
            y = float(self.y_positions[self.index])
            pitch = float(self.pitch_angles[self.index])
            roll = float(self.roll_angles[self.index])

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [
                'world_to_sphere_x',
                'x_to_y',
                'sphere_to_cylinder_pitch',
                'pitch_to_cylinder_roll'
            ]
            joint_state.position = [x , y, pitch, roll]
            self.joint_state_publisher.publish(joint_state)
            self.index +=1

        else:
            self.get_logger().info("ALl states have published")
    
    def publish_obstacle_marker(self):
        # Calculate obstacle positions
        obs1_x = self.amplitude * np.sin(2 * np.pi * self.frequency * self.t) + 1
        obs1_y = 1.5

        # Create a marker for the obstacle
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obs1_x
        marker.pose.position.y = obs1_y
        marker.pose.position.z = 0.0# Assume obstacle height is 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # Radius of the obstacle
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0  # Blue

        self.marker_publisher.publish(marker)



        obs2_x = self.amplitude * np.sin(2 * np.pi * self.frequency * self.t) + 3
        obs2_y = 1.5

        # Create a marker for the obstacle
        marker2 = Marker()
        marker2.header.frame_id = "world"
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.ns = "obstacles"
        marker2.id = 1
        marker2.type = Marker.CUBE
        marker2.action = Marker.ADD
        marker2.pose.position.x = obs2_x
        marker2.pose.position.y = obs2_y
        marker2.pose.position.z = 0.0# Assume obstacle height is 0.5
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = 0.3  # Radius of the obstacle
        marker2.scale.y = 0.3
        marker2.scale.z = 0.3
        marker2.color.a = 1.0  # Alpha
        marker2.color.r = 1.0  # Red
        marker2.color.g = 0.0
        marker2.color.b = 0.0  # Blue

        self.marker_publisher.publish(marker2)


        marker3 = Marker()
        marker3.header.frame_id = "world"
        marker3.header.stamp = self.get_clock().now().to_msg()
        marker3.ns = "Goal Point"
        marker3.id = 100
        marker3.type = Marker.SPHERE
        marker3.action = Marker.ADD
        marker3.pose.position.x = 1.2
        marker3.pose.position.y = 2.0
        marker3.pose.position.z = 0.0# Assume obstacle height is 0.5
        marker3.pose.orientation.x = 0.0
        marker3.pose.orientation.y = 0.0
        marker3.pose.orientation.z = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.scale.x = 0.1  # Radius of the obstacle
        marker3.scale.y = 0.1
        marker3.scale.z = 0.1
        marker3.color.a = 1.0  # Alpha
        marker3.color.r = 0.5  # Red
        marker3.color.g = 0.2
        marker3.color.b = 0.1  # Blue

        self.marker_publisher.publish(marker3)

        marker4 = Marker()
        marker4.header.frame_id = "world"
        marker4.header.stamp = self.get_clock().now().to_msg()
        marker4.ns = "Starting POint"
        marker4.id = 200
        marker4.type = Marker.SPHERE
        marker4.action = Marker.ADD
        marker4.pose.position.x = 0.0
        marker4.pose.position.y = 0.0
        marker4.pose.position.z = 0.0# Assume obstacle height is 0.5
        marker4.pose.orientation.x = 0.0
        marker4.pose.orientation.y = 0.0
        marker4.pose.orientation.z = 0.0
        marker4.pose.orientation.w = 1.0
        marker4.scale.x = 0.1  # Radius of the obstacle
        marker4.scale.y = 0.1
        marker4.scale.z = 0.1
        marker4.color.a = 1.0  # Alpha
        marker4.color.r = 0.0  # Red
        marker4.color.g = 1.0
        marker4.color.b = 0.0  # Blue

        self.marker_publisher.publish(marker4)

        self.t += 0.1  # Increment time step



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