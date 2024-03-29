from setuptools import setup
import os
from glob import glob

package_name = 'joyride_servers'

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
    description='Servers and service providers for Joyride self-driving vehicle.',
    license='LGPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dbw_usb_link_node = joyride_servers.dbw_usb_link_node:main',
            'joyride_static_tf_broadcaster = joyride_servers.joyride_static_tf_broadcaster:main',
            'automode_manager = joyride_servers.automode_manager:main',
            'computer_monitor = joyride_servers.computer_monitor:main',
            'serial_link = joyride_servers.serial_link:main'
        ],
    },
)
