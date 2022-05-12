import os
from glob import glob
from setuptools import setup

package_name = 'perception_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name, 
        'perception_service.numpify',
        'perception_service.geom',
        'perception_service.algorithms',
        'perception_service.models',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='Juan Miguel jimeno',
    maintainer_email='jimenojmm@gmail.com',
    description='ROS2 Robot Arm Driver',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'camera_transform_publisher = perception_service.camera_transform_publisher:main',
            'perception_server = perception_service.perception_server:main',
            'perception_client = perception_service.perception_client:main'
        ],
    },
)
