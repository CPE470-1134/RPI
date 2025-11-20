from setuptools import find_packages, setup

package_name = 'camera_pkg'

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
    maintainer='root',
    maintainer_email='cpe470.1134@gmail.com',
    description='Camera publisher and subscriber nodes for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node=camera_pkg.camera_node:main',
            'camera_publisher=camera_pkg.camera_publisher:main',
            'camera_subscriber=camera_pkg.camera_subscriber:main',
            'calibration=camera_pkg.calibration:main'
        ],
    },
)
