from setuptools import find_packages, setup

package_name = 'aruco_pkg'

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
    description='ArUco marker detection and pose estimation nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_pose_node=aruco_pkg.aruco_pose_node:main',
            'aruco_node=aruco_pkg.aruco_node:main',
        ],
    },
)
