from setuptools import setup, find_packages
import os
import glob
package_name = 'joyride_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), #Add Submodules
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max DeSantis',
    maintainer_email='max.desantis@okstate.edu',
    description='Perception algorithms for Joyride self-driving vehicle.',
    license='LGPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blob_detector = joyride_perception.blob_detector:main',
            'lane_detector = joyride_perception.lane_detector:main',
            'pinhole_test = joyride_perception.pinhole_test:main',
            'optical_transform_calibrator = joyride_perception.camera_transform_calibrator:main',
            'sign_detector = joyride_perception.traffic_sign_detector:main',
            'obstacle_detector = joyride_perception.obstacle_detector:main',
            'uv_to_pointcloud = joyride_perception.uv_to_pointcloud:main'
        ],
    },
)
