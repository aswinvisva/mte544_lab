from setuptools import setup

package_name = 'ros2_lab1_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aswin',
    maintainer_email='aswin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_encoder_odom = ros2_lab1_5.wheel_encoder_odom:main',
            'talker = ros2_lab1_5.turtlebot_circle_trajectory:main'
        ],
    },
)
