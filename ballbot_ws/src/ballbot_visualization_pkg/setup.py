
from setuptools import find_packages, setup

setup(
    name='ballbot_visualization_pkg',
    version='0.0.0',
    packages=find_packages(where='.', exclude=['test']),  # Finds all packages, including nested
    package_dir={'': '.'},  # Root directory is the base for the package
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ballbot_visualization_pkg']),
        ('share/ballbot_visualization_pkg', ['package.xml']),
        ('share/ballbot_visualization_pkg/launch', ['launch/ballbot_visualization.launch.py']),
        ('share/ballbot_visualization_pkg/urdf', ['urdf/ballbot.urdf']),
    ],
    install_requires=[
        'setuptools',
        'transforms3d',  # Include any additional dependencies here
    ],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='lawrence',
    maintainer_email='lawrence@todo.todo',
    description='A package for visualizing ballbot trajectories and states in RViz2.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'state_publisher = ballbot_visualization_pkg.state_publisher:main',
        ],
    },
)

# from setuptools import find_packages, setup

# setup(
#     name='ballbot_visualization_pkg',
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages', ['resource/ballbot_visualization_pkg']),
#         ('share/ballbot_visualization_pkg', ['package.xml']),
#         ('share/ballbot_visualization_pkg/launch', ['launch/ballbot_visualization.launch.py']),
#         ('share/ballbot_visualization_pkg/urdf', ['urdf/ballbot.urdf']),
#     ],
#     install_requires=[
#         'setuptools',
#         'transforms3d',  # Add this line
#     ],
#     extras_require={
#         'test': ['pytest'],
#     },
#     zip_safe=True,
#     maintainer='lawrence',
#     maintainer_email='lawrence@todo.todo',
#     description='A package for visualizing ballbot trajectories and states in RViz2.',
#     license='Apache License 2.0',
#     entry_points={
#         'console_scripts': [
#             'state_publisher = ballbot_visualization_pkg.state_publisher:main',
#         ],
#     },
# )
