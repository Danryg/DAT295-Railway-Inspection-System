from setuptools import setup
import os
from glob import glob

package_name = 'rail_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ning',
    maintainer_email='None@example.com',
    description='Rail crack detection package using LaserScan for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rail_scan_detect = rail_detection.rail_crack_detection_laserscan:main',
        ],
    },
)