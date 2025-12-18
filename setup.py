from glob import glob

from setuptools import find_packages, setup

package_name = 'mapir_camera_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duda Andrada',
    maintainer_email='duda.andrada@isr.uc.pt',
    description='ROS 2 driver node for MAPIR Survey3 camera',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = mapir_camera_ros2.camera_node:main',
            'indices_node = mapir_camera_ros2.indices_node:main',
        ],
    },
)
