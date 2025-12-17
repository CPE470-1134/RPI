from setuptools import setup

package_name = 'architecture'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Robot mission architecture',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = architecture.aruco_node:main',
            'controller_node = architecture.controller_node:main',
            'director_node = architecture.director_node:main',
            'driver_node = architecture.driver_node:main',
            'lidar_node = architecture.lidar_node:main',
            'localizer_node = architecture.localizer_node:main',
            'safety_node = architecture.safety_node:main',
        ],
    },
)