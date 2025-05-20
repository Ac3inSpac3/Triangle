from setuptools import find_packages, setup

package_name = 'triangle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch and config
        (f'share/{package_name}/launch',  ['launch/robot.launch.py']),
        (f'share/{package_name}/launch',  ['launch/nav.launch.py']),
        (f'share/{package_name}/launch',  ['launch/manual_robot.launch.py']),
        (f'share/{package_name}/launch',  ['launch/manual_robot_logging.launch.py']),
        (f'share/{package_name}/config',  ['config/bno055_params_i2c.yaml']),
        (f'share/{package_name}/config',  ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joshr',
    maintainer_email='joshroe2001@gmail.com',
    description='Bringup for Triangle holonomic robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # ... your existing entry points ...
        'teleop_joy_node = triangle.teleop_joy_node:main',
        'log_odometry = triangle.log_odometry:main',
        ],
    },
)
