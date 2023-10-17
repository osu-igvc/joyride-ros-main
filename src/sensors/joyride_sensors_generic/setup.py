from setuptools import setup, find_packages

package_name = 'joyride_sensors_generic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'raw_image_publisher = joyride_sensors_generic.raw_image_publisher:main'
        ],
    },
)
