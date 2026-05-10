from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rh8d_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labauto',
    maintainer_email='ehtishamashraf67@gmail.com',
    description='TODO: ROS2 Control package for RH8D hand',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rh8dL_gestures = rh8d_control.rh8dL_gestures:main', 
            'rh8dL_object_pick = rh8d_control.rh8dL_object_pick:main',
            'rh8dL_object_pick_pos = rh8d_control.rh8dL_object_pick_pos:main',
        ],
    },
)
