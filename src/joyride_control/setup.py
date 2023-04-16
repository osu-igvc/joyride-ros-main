from setuptools import setup

package_name = 'joyride_control'

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
    maintainer='Max DeSantis',
    maintainer_email='max.desantis@okstate.edu',
    description='Actuator control nodes for Joyride self-driving vehicle.',
    license='LGPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_preprocess_node = joyride_control.vel_preprocessor:main'
        ],
    },
)
