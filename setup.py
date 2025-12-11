from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/controller.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ias',
    maintainer_email='ias@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = controller.keyboard_control:main',
            'realsense_imu_logger = controller.realsense_imu_logger:main',
            'sabertooth_cmd_vel_bridge = controller.sabertooth_cmd_vel_bridge:main',
            'wheel_encoder = controller.wheel_encoder:main',
            'wheel_odom_publisher = controller.wheel_odom_publisher:main',
        ],
    },
)
