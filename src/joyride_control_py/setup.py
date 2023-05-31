from setuptools import setup

package_name = 'joyride_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joyride-obc',
    maintainer_email='max.desantis@okstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_preprocessor = joyride_control_py.vel_preprocessor:main',
            'simple_go_to_pose = joyride_control_py.simple_go_to_pose:main'
        ],
    },
)
