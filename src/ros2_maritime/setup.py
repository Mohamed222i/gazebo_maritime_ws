from setuptools import find_packages, setup
from glob import glob
package_name = 'ros2_maritime'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # Include launch files
        ('share/' + package_name + '/urdf', glob('urdf/*.*')),  # Include URDF files 
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')), # Include Rviz2 configs
        ('share/' + package_name + '/gz', glob('gz/*.sdf')), #Include SDF for Gazebo files
        ('share/' + package_name + '/meshes' , glob('meshes/*.*')), #all mesh files
        ('share/' + package_name + '/config' ,glob('config/*.yaml')) # config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='farouk9519@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altimeter_converter = ros2_maritime.altimeter_converter:main',
            'magnetometer_converter = ros2_maritime.magnetometer_converter:main'
        ],
    },
)
