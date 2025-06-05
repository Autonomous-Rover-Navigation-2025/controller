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
            'serial_com = controller.serial_com:main',
            'wheel_odometry = controller.wheel_odometry:main',
            'keyboard_control = controller.keyboard_control:main',
            'test_serial_com = controller.test_serial_com:main',
            'sabertooth_cmd_vel_bridge = controller.sabertooth_cmd_vel_bridge:main',
            'odom_publisher = controller.odom_publisher:main',
            'imu_publisher = controller.imu_publisher:main',
        ],
    },
)
