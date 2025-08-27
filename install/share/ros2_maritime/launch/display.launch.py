from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    gazebo_world_path = "/home/pc/gazebo_maritime_ws/src/gazebo_maritime/worlds/sydney_regatta.sdf"
    urdf_path = "/home/pc/gazebo_maritime_ws/src/gazebo_maritime/models/wam-v/model.sdf"
    bridge_config_path = "/home/pc/gazebo_maritime_ws/src/ros2_maritime/config/bridge_config.yaml"

    # Load bridge config
    with open(bridge_config_path, 'r') as f:
        bridge_config = yaml.safe_load(f)

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=["gz", "sim", gazebo_world_path],
            output="screen"
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
            output='screen',
            shell=True
        ),


        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': open(urdf_path).read()},
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}
            ]
        ),


        Node(
            package='ros2_maritime',
            executable='altimeter_converter',
            name='altimeter_converter'
        ),
        Node(
            package='ros2_maritime',
            executable='magnetometer_converter',
            name='magnetometer_converter'
        ),
        Node(
            package='ros2_maritime',
            executable='imu_covariance',
            name='imu_covariance'
        ),



        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'wam-v/base_link/imu_sensor']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'left_engine_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'right_engine_link']
        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'wam-v/base_link/navsat_sensor']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'wam-v/base_link/altimeter_sensor']
        ),
        



        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ '/home/pc/gazebo_maritime_ws/src/ros2_maritime/config/ekf_filter.yaml' ]
        ),
       


        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=['/home/pc/gazebo_maritime_ws/src/ros2_maritime/config/navsat.yaml'],
            remappings=[
                ('/gps/fix', '/navsat'),
                ('/imu/data', '/sensor/imu'), 
                ('/odometry/filtered', '/odometry/filtered')  
            ]
        ),



        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),





        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config_path}]
        )
    ])