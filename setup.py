from setuptools import setup

package_name = 'mapir_camera_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapir_camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duda Andrada',
    maintainer_email='duda.andrada@isr.uc.pt',
    description='ROS 2 driver node for MAPIR Survey3 camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = mapir_camera_ros2.camera_node:main',
        ],
    },
)
