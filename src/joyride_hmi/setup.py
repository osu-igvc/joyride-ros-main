from setuptools import setup
from glob import glob
import os

package_name = 'joyride_hmi'
ui_name = 'window.ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + ui_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'style.qss']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OSU Team Joyride',
    maintainer_email='max.desantis@okstate.edu',
    description='Human-Machine Interfaces for Joyride Autonomous Car',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = joyride_hmi.dashboard_node:main',
            'joystick_pub_node = joyride_hmi.joystick_pub_node:main',
            'joystick_mapper_node = joyride_hmi.joystick_mapper_node:main',
            'tts_node = joyride_hmi.tts_node:main'
        ],
    },
)
