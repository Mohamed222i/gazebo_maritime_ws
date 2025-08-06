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
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=['/home/pc/gazebo_maritime_ws/src/ros2_maritime/config/navsat.yaml'],
            remappings=[
                ('/gps/fix', '/navsat')
            ]
        ),





        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'frequency': 30.0},
                {'sensor_timeout': 2.0},
                {'two_d_mode': False},     # ENABLE THESE CRITICAL SETTINGS
                {'publish_acceleration': True},
                {'publish_transform': True},
                {'print_diagnostics': True},
                {'use_control': False},
                {'odom0': '/odometry/gps'},
                {'imu0': '/imu'},
                {'imu1': '/Magnetometer'},
                {'pose0': '/altimeter'},

                {'imu0_differential': False},
                {'imu0_remove_gravitational_acceleration': True},
                {'imu0_orientation_covariance': 0.01},
                {'imu0_angular_velocity_covariance': 0.001},
                {'imu0_linear_acceleration_covariance': 0.01},

                {'imu1_differential': False},
                {'imu1_orientation_covariance': 0.05,},

                {'pose0_pose_covariance_diagonal': [0.0, 0.0, 0.1] },

                {'odom0_differential': False},
                {'odom0_relative': False},
                {'odom0_pose_covariance_diagonal': [4.0, 4.0, 0.0] },
                {'odom0_twist_covariance_diagonal': [0.5, 0.5, 0.0] },
                
                # Marine-Specific Parameters
                {'acceleration_limits': [2.0, 2.0, 1.0] },
                {'deceleration_limits': [2.0, 2.0, 1.0] },
                {'baro_delay': 0.2 },
                {'baro_vertical_position_threshold': 0.5 },




                {'odom0_config': [
                    True, True, False,
                    False, False, False, 
                    False, False, False, 
                    False, False, False, 
                    False, False, False
                ]},                
                {'imu0_config': [
                    False, False, False,   # Position (x,y,z) - unused
                    True,  True,  True,    # Roll, Pitch, Yaw (accel/mag primary)
                    False, False, False,   # Velocity (x,y,z) - unused
                    True,  True,  True,    # Angular velocity (x,y,z) - secondary for orientation
                    True,  True,  True     # Acceleration (x,y,z) - secondary for velocity
                ]},

                {'imu1_config': [
                    False, False, False,   
                    False, False,  True,    # Yaw (accel/mag primary)
                    False, False, False,   
                    False, False, False,   
                    False, False, False     
                ]},
                {'pose0_config': [
                    False, False, True,   # Position (x,y,z) 
                    False, False, False,    
                    False, False, False,    
                    False, False, False     
                ]},  
                {'publish_tf': True},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'map'},    ## map or odom ????
                {'qos_overrides./imu.reliability': 'reliable'},
                {'qos_overrides./navsat.reliability': 'reliable'},
                {'qos_overrides./demo/odom.reliability': 'reliable'},
                {'process_noise_covariance': [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}
            ],
            remappings=[
                ('/gps/fix', '/navsat')
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