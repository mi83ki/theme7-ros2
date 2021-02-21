from setuptools import setup

package_name = 'find_wally'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/find_wally_only_robo.launch.py']),
        ('share/' + package_name, ['launch/find_wally_robo.launch.py']),
        ('share/' + package_name, ['launch/find_wally_pc.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='mr.teruterubouz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_wally = find_wally.find_wally:main',
            'moving_controller = find_wally.moving_controller:main',
            'find_wally_yolo = find_wally.find_wally_yolo:main'
        ],
    },
)
