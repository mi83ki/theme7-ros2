from setuptools import setup

package_name = 'my_ros2serial_python'

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
    maintainer='Miyazaki',
    maintainer_email='mr.teruterubouz@gmail.com',
    description='Serial comunication node with arduino for ROS2',
    license='Version 0.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = my_ros2serial_python.serial_node:main'
        ],
    },
)
