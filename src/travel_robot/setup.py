from setuptools import find_packages, setup

package_name = 'travel_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/world.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/ethiopia_relaxed.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'robot_planner = travel_robot.robot_planner:main',
        'vision_navigation = travel_robot.vision_navigation:main',
        'sensor_fusion = travel_robot.sensor_fusion:main',
        'obstacle_avoidance = travel_robot.obstacle_avoidance:main',
        'path_follower = travel_robot.path_follower:main',
    ],
},
)
