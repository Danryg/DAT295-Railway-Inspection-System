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

        (os.path.join('share', package_name, 'models/turtlebot3_burger_velodyne'), glob(os.path.join('worlds/models/turtlebot3_burger_velodyne', '*.[sdf]*'))),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_velodyne'), glob(os.path.join('worlds/models/turtlebot3_waffle_velodyne', '*.[sdf]*'))),


        #new for turtlebot
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.[urdf]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.[model]*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[pgm]*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yaml]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.[yaml]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.[rviz]*'))),

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
            'speed = test_world_gazebo.speed:main',
            'input = test_world_gazebo.input_handler:main',
            'controller = test_world_gazebo.speed_controller:main',
            'dock = test_world_gazebo.movetoamp:main',
            'undock = test_world_gazebo.undock:main'
        ],
    },
)
