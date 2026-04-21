from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sensor_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Launch and serial bringup for Leviathan sensors.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ads1115_water_quality_publisher = sensor_bringup.ads1115_water_quality_publisher:main',
            'serial_json_publisher = sensor_bringup.serial_json_publisher:main',
        ],
    },
)
