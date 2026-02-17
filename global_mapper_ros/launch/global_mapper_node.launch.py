#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    # launch‐time arguments
    quad                   = LaunchConfiguration('quad')
    load_params            = LaunchConfiguration('load_params')
    param_file             = LaunchConfiguration('param_file')
    depth_pointcloud_topic      = LaunchConfiguration('depth_pointcloud_topic')
    pose_topic             = LaunchConfiguration('pose_topic')
    goal_topic             = LaunchConfiguration('goal_topic')
    odom_topic             = LaunchConfiguration('odom_topic')
    occupancy_grid_topic   = LaunchConfiguration('occupancy_grid_topic')
    unknown_grid_topic     = LaunchConfiguration('unknown_grid_topic')
    frontier_grid_topic    = LaunchConfiguration('frontier_grid_topic')
    distance_grid_topic    = LaunchConfiguration('distance_grid_topic')
    cost_grid_topic        = LaunchConfiguration('cost_grid_topic')
    path_topic             = LaunchConfiguration('path_topic')
    # dynamic_grid_topic   = LaunchConfiguration('dynamic_grid_topic')
    sparse_path_topic      = LaunchConfiguration('sparse_path_topic')

    param_file_launch_arg = DeclareLaunchArgument('param_file',           default_value='flightgoggles.yaml',           description='name of your params file (in cfg/)')

    # where to find your .yaml (make sure you install it; see below)
    param_file_path = PathJoinSubstitution([
        FindPackageShare('global_mapper_ros'),
        'cfg',
        param_file
    ])

    return LaunchDescription([
        #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        DeclareLaunchArgument('quad',                 default_value='NX01',                        description='name of the quad / ros namespace'),
        DeclareLaunchArgument('load_params',          default_value='true',                        description='whether to load the YAML params'),
        DeclareLaunchArgument('depth_pointcloud_topic', default_value='mid360_PointCloud2',  description='input pointcloud topic'),
        # DeclareLaunchArgument('depth_pointcloud_topic', default_value='d435/depth/color/points',  description='input pointcloud topic'),
        DeclareLaunchArgument('pose_topic',           default_value='mavros/local_position/pose',   description='input pose topic'),
        DeclareLaunchArgument('goal_topic',           default_value='/move_base_simple/goal',       description='input goal topic'),
        DeclareLaunchArgument('odom_topic',           default_value='odometry/filtered_no',         description='input odometry topic'),
        # **no more "~" here**:
        DeclareLaunchArgument('occupancy_grid_topic', default_value='occupancy_grid',              description='output occupancy grid topic'),
        DeclareLaunchArgument('unknown_grid_topic',   default_value='unknown_grid',                description='output unknown grid topic'),
        DeclareLaunchArgument('frontier_grid_topic',  default_value='frontier_grid',               description='output frontier grid topic'),
        DeclareLaunchArgument('distance_grid_topic',  default_value='distance_grid',               description='output distance grid topic'),
        DeclareLaunchArgument('cost_grid_topic',      default_value='cost_grid',                   description='output cost grid topic'),
        DeclareLaunchArgument('path_topic',           default_value='path',                        description='output path topic'),
        # DeclareLaunchArgument('dynamic_grid',           default_value='dynamic_grid',                        description='output dynamic grid topic'),
        DeclareLaunchArgument('sparse_path_topic',    default_value='sparse_path',                 description='output sparse path topic'),
        param_file_launch_arg,

        #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        Node(
            package='global_mapper_ros',
            executable='global_mapper_node',
            name='global_mapper_ros',
            namespace=quad,
            output='screen',
            # only load params if you really want—here we always load
            parameters=[param_file_path],
            remappings=[
                ('depth_pointcloud_topic',    depth_pointcloud_topic),
                ('pose_topic',           pose_topic),
                ('goal_topic',           goal_topic),
                ('odom_topic',           odom_topic),
                ('occupancy_grid_topic', occupancy_grid_topic),
                ('unknown_grid_topic',   unknown_grid_topic),
                ('frontier_grid_topic',  frontier_grid_topic),
                ('distance_grid_topic',  distance_grid_topic),
                ('cost_grid_topic',      cost_grid_topic),
                ('path_topic',           path_topic),
                # ('dynamic_grid_topic',   dynamic_grid_topic),
                ('sparse_path_topic',    sparse_path_topic),
            ],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
            ),
    ])
