#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    sensor_offset_x_arg = DeclareLaunchArgument(
        'sensorOffsetX',
        default_value='0.0',
        description='Sensor offset in X direction'
    )
    
    sensor_offset_y_arg = DeclareLaunchArgument(
        'sensorOffsetY',
        default_value='0.0',
        description='Sensor offset in Y direction'
    )
    
    camera_offset_z_arg = DeclareLaunchArgument(
        'cameraOffsetZ',
        default_value='0.0',
        description='Camera offset in Z direction'
    )
    
    two_way_drive_arg = DeclareLaunchArgument(
        'twoWayDrive',
        default_value='false',
        description='Enable two-way driving'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'maxSpeed',
        default_value='1.0',
        description='Maximum speed'
    )
    
    autonomy_mode_arg = DeclareLaunchArgument(
        'autonomyMode',
        default_value='true',
        description='Enable autonomy mode'
    )
    
    autonomy_speed_arg = DeclareLaunchArgument(
        'autonomySpeed',
        default_value='1.0',
        description='Autonomy speed'
    )
    
    joy_to_speed_delay_arg = DeclareLaunchArgument(
        'joyToSpeedDelay',
        default_value='2.0',
        description='Joy to speed delay'
    )
    
    goal_x_arg = DeclareLaunchArgument(
        'goalX',
        default_value='0.0',
        description='Goal X coordinate'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goalY',
        default_value='0.0',
        description='Goal Y coordinate'
    )

    # Local Planner Node
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder': PathJoinSubstitution([FindPackageShare('local_planner'), 'paths']),
            'vehicleLength': 0.6,
            'vehicleWidth': 0.6,
            'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY': LaunchConfiguration('sensorOffsetY'),
            'twoWayDrive': LaunchConfiguration('twoWayDrive'),
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': True,
            'checkObstacle': True,
            'checkRotObstacle': False,
            'adjacentRange': 4.25,
            'obstacleHeightThre': 0.2,
            'groundHeightThre': 0.1,
            'costHeightThre': 0.1,
            'costScore': 0.02,
            'useCost': False,
            'pointPerPathThre': 2,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'maxSpeed': LaunchConfiguration('maxSpeed'),
            'dirWeight': 0.02,
            'dirThre': 90.0,
            'dirToVehicle': False,
            'pathScale': 1.25,
            'minPathScale': 0.75,
            'pathScaleStep': 0.25,
            'pathScaleBySpeed': True,
            'minPathRange': 1.0,
            'pathRangeStep': 0.5,
            'pathRangeBySpeed': True,
            'pathCropByGoal': True,
            'autonomyMode': LaunchConfiguration('autonomyMode'),
            'autonomySpeed': LaunchConfiguration('autonomySpeed'),
            'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay'),
            'joyToCheckObstacleDelay': 5.0,
            'goalClearRange': 0.5,
            'goalX': LaunchConfiguration('goalX'),
            'goalY': LaunchConfiguration('goalY'),
        }]
    )

    # Path Follower Node
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY': LaunchConfiguration('sensorOffsetY'),
            'pubSkipNum': 1,
            'twoWayDrive': LaunchConfiguration('twoWayDrive'),
            'lookAheadDis': 0.5,
            'yawRateGain': 7.5,
            'stopYawRateGain': 7.5,
            'maxYawRate': 20.0,
            'maxSpeed': LaunchConfiguration('maxSpeed'),
            'maxAccel': 2.5,
            'switchTimeThre': 1.0,
            'dirDiffThre': 0.1,
            'stopDisThre': 0.2,
            'slowDwnDisThre': 0.85,
            'useInclRateToSlow': False,
            'inclRateThre': 120.0,
            'slowRate1': 0.25,
            'slowRate2': 0.5,
            'slowTime1': 2.0,
            'slowTime2': 2.0,
            'useInclToStop': False,
            'inclThre': 45.0,
            'stopTime': 5.0,
            'noRotAtStop': False,
            'noRotAtGoal': True,
            'autonomyMode': LaunchConfiguration('autonomyMode'),
            'autonomySpeed': LaunchConfiguration('autonomySpeed'),
            'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay'),
        }]
    )

    # Note: The commented static transform publishers from the original XML are not included
    # as they were commented out. If needed, they can be added using:
    # from launch_ros.actions import StaticTransformBroadcaster
    # or using tf2_ros static_transform_publisher node

    return LaunchDescription([
        # Launch arguments
        sensor_offset_x_arg,
        sensor_offset_y_arg,
        camera_offset_z_arg,
        two_way_drive_arg,
        max_speed_arg,
        autonomy_mode_arg,
        autonomy_speed_arg,
        joy_to_speed_delay_arg,
        goal_x_arg,
        goal_y_arg,
        
        # Nodes
        local_planner_node,
        path_follower_node,
    ])