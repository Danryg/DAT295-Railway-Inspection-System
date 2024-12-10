import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'test_world_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.[world]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('worlds/models/amp', '*.[sdf]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('worlds/models/amp_hull', '*.[sdf]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('worlds/models/amp_axle', '*.[sdf]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abishek',
    maintainer_email='abishek.swaminathan03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
