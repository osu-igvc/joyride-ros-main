from setuptools import setup
import os
from glob import glob

package_name = 'joyride_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igvcsp2022',
    maintainer_email='max.desantis@okstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clock_bag_repub = joyride_core.bag_clock_replay:main',
            'imu_tf_pub = joyride_core.imu_tf_test:main',
            'diag_test = joyride_core.diagnostic_test:main'
        ],
    },
)
