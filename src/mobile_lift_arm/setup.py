from setuptools import setup

package_name = 'scissor_lift_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scissor_lift.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/scissor_lift.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/scissor_lift_controllers.yaml']),
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
