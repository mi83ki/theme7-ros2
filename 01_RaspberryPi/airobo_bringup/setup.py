from setuptools import setup

package_name = 'airobo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/airobo_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miyazaki',
    maintainer_email='mr.teruterubouz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airobo_bringup = airobo_bringup.airobo_bringup:main'
        ],
    },
)
