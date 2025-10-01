from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'scara'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")), #
        (os.path.join("share", package_name,'models'), glob("models/*.*")), #hacer visible todo lo que este en models
        (os.path.join("share", package_name,'config'), glob("config/*.yaml")),
        (os.path.join("share", package_name,'cad_models'), glob("cad_models/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camila',
    maintainer_email='camila@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "inverse_kinematics= scara.inverse_kinematics:main",
            "direct_kinematics= scara.direct_kinematics:main",
            "translate_node= scara.translate_node:main",
            "trayectory_planner= scara.trayectory_planner:main",
            "dxf_parser_node = scara.dxf_parser_node:main",
        ],
    },
)
