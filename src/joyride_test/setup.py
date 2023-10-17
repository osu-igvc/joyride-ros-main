from setuptools import setup
from glob import glob
import os

package_name = 'joyride_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max DeSantis',
    maintainer_email='max.desantis@okstate.edu',
    description='Internal testing and validation nodes for Joyride self-driving vehicle.',
    license='LGPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
