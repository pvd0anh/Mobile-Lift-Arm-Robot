from setuptools import setup

# Package metadata
package_name = 'scissor_lift_control'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        # Resource index used by ament
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install the package manifest
        ('share/' + package_name, ['package.xml']),
        # Install launch and config files into the share directory
        ('share/' + package_name + '/launch', ['scissor_lift_control/launch/scissor_lift.launch.py']),
        ('share/' + package_name + '/config', ['scissor_lift_control/config/scissor_lift_controllers.yaml']),
        ('share/' + package_name + '/config', ['scissor_lift_control/config/ros2_controllers.yaml']),
        # Install URDF files (if present) into the share directory
        # You would add actual URDF/xacro files here when the model is complete
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 scissor lift simulation and control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scissor_lift_controller = scissor_lift_control.controller:main',
            'scissor_lift_gui = scissor_lift_control.gui:main',
        ],
    },
)