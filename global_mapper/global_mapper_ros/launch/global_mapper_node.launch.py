from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
   
   return LaunchDescription([
      # Declare the quad argument with a default value
      DeclareLaunchArgument('quad', default_value='SQ01s', description='Name of the quad'),

      # Declare the parameters for loading
      DeclareLaunchArgument('load_params', default_value='true', description='Load parameters'),
      DeclareLaunchArgument('param_file', default_value='global_mapper.yaml', description='Global mapper param file'),

      # Declare input topics
      DeclareLaunchArgument('depth_image_topic', default_value='camera/depth/image_rect_raw', description='Depth image topic'),
      DeclareLaunchArgument('pose_topic', default_value='state', description='Pose topic'),
      DeclareLaunchArgument('goal_topic', default_value='/move_base_simple/goal', description='Goal topic'),
      DeclareLaunchArgument('odom_topic', default_value='odometry/filtered_no', description='Odometry topic'),

      # Declare output topics
      DeclareLaunchArgument('occupancy_grid_topic', default_value='~occupancy_grid', description='Occupancy grid topic'),
      DeclareLaunchArgument('unknown_grid_topic', default_value='~unknown_grid', description='Unknown grid topic'),
      DeclareLaunchArgument('frontier_grid_topic', default_value='~frontier_grid', description='Frontier grid topic'),
      DeclareLaunchArgument('distance_grid_topic', default_value='~distance_grid', description='Distance grid topic'),
      DeclareLaunchArgument('cost_grid_topic', default_value='~cost_grid', description='Cost grid topic'),
      DeclareLaunchArgument('path_topic', default_value='~path', description='Path topic'),
      DeclareLaunchArgument('sparse_path_topic', default_value='~sparse_path', description='Sparse path topic'),

      Node(
         package='global_mapper_ros',
         executable='global_mapper_ros',
         name='global_mapper_ros', 
         namespace=LaunchConfiguration('quad'),
         output='screen',
         parameters=[{
            'param_file': LaunchConfiguration('param_file')
         }] if LaunchConfiguration('load_params') == 'true' else [],
         remappings=[
            # input remaps
            ('~depth_image_topic', LaunchConfiguration('depth_image_topic')),
            ('~pose_topic', LaunchConfiguration('pose_topic')),
            ('~goal_topic', LaunchConfiguration('goal_topic')),
            ('~odom_topic', LaunchConfiguration('odom_topic')),

            # output remaps
            ('~occupancy_grid_topic', LaunchConfiguration('occupancy_grid_topic')),
            ('~unknown_grid_topic', LaunchConfiguration('unknown_grid_topic')),
            ('~frontier_grid_topic', LaunchConfiguration('frontier_grid_topic')),
            ('~distance_grid_topic', LaunchConfiguration('distance_grid_topic')),
            ('~cost_grid_topic', LaunchConfiguration('cost_grid_topic')),
            ('~path_topic', LaunchConfiguration('path_topic')),
            ('~sparse_path_topic', LaunchConfiguration('sparse_path_topic'))
         ]
      ),
   ])