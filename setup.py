from setuptools import find_packages, setup

package_name = 'scara_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scara_publisher_launch.py']),
        ('share/' + package_name + '/config', ['config/scara_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='maintainer@example.com',
    description='A ROS2 package for SCARA robot publisher functionality',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scara_publisher_node = scara_publisher.scara_publisher_node:main',
            'scara_controller_node = scara_publisher.scara_controller_node:main',
        ],
    },
)