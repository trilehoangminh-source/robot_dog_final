from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_dog_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.wbt'))),
        (os.path.join('share', package_name, 'protos'), glob(os.path.join('protos', '*.proto'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minhtri',
    maintainer_email='tri.lehoangminh@hcmut.edu.vn',
    description='Robot quadruped simulation with SMC controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smc_controller_node = robot_dog_final.smc_controller_node:main',
        ],
    },
)