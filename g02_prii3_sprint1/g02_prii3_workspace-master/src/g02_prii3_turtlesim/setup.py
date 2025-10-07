from setuptools import setup
import os
from glob import glob

package_name = 'g02_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laura',
    maintainer_email='laura@example.com',
    description='TurtleSim controller for group number drawing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = g02_prii3_turtlesim.turtle_controller:main',
        ],
    },
)
