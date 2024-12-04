import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('ballbot_visualization_pkg'),
        'urdf',
        'ballbot.urdf'  # Make sure the URDF filename matches
    )

    # Load the URDF content
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Launch description
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # Static Transform Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',  # Translation
                '0', '0', '0', '1',  # Quaternion
                'map', 'world'  # Parent and child frames
            ]
        ),
        # State Publisher Node
        Node(
            package='ballbot_visualization_pkg',
            executable='state_publisher',
            name='state_publisher',
            output='screen'
        ),
    ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Path to the URDF file
#     urdf_file_path = os.path.join(
#         get_package_share_directory('ballbot_visualization_pkg'),
#         'urdf',
#         'ballbot.urdf'
#     )

#     # Load the URDF content
#     with open(urdf_file_path, 'r') as urdf_file:
#         robot_description_content = urdf_file.read()

#     # Launch description
#     return LaunchDescription([
#         # Robot State Publisher
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': robot_description_content}]
#         ),
#         # Static Transform Publisher
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_transform_publisher',
#             output='screen',
#             arguments=[
#                 '--x', '0', '--y', '0', '--z', '0',  # Translation
#                 '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',  # Quaternion
#                 '--frame-id', 'map', '--child-frame-id', 'world'  # Parent and child frames
#             ]
#         ),
#         # State Publisher Node
#         Node(
#             package='ballbot_visualization_pkg',
#             executable='state_publisher',
#             name='state_publisher',
#             output='screen'
#         ),
#     ])
