from setuptools import find_packages, setup

package_name = 'iris_odometry_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'rclpy', 'px4_msgs', 'nav_msgs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthuralexandersson@hotmail.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_converter = iris_odometry_converter.odometry_converter:main'
        ],
    },
)
