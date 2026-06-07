import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

def convert_str_to_bool(str):
    return True if (str == 'true' or str == 'True' or str == 1 or str == '1') else False

def generate_launch_description():
    
    # declare args 
    namespace_arg = DeclareLaunchArgument('namespace', default_value='NX01', description='Namespace of the nodes')
    param_file_arg = DeclareLaunchArgument('param_file', default_value='flightgoggles.yaml', description='name of param file')
    use_tracker_arg = DeclareLaunchArgument('use_tracker', default_value='true', description='Whether to use obstacle tracker or not')
    publish_init_tf_arg = DeclareLaunchArgument('publish_init_tf', default_value='true', description='Whether to publish the transform between initial vehicle pose and world or not')
    pc_arg = DeclareLaunchArgument('depth_pointcloud_topic', default_value='mid360_PointCloud2',  description='input pointcloud topic')
    pose_topic_arg = DeclareLaunchArgument('pose_topic',           default_value='dlio/odom_node/pose',   description='input pose topic')
    goal_topic_arg = DeclareLaunchArgument('goal_topic',           default_value='/move_base_simple/goal',       description='input goal topic')
    odom_topic_arg = DeclareLaunchArgument('odom_topic',           default_value='odometry/filtered_no',         description='input odometry topic')
    occ_grid_topic_arg = DeclareLaunchArgument('occupancy_grid_topic', default_value='occupancy_grid',              description='output occupancy grid topic')
    unk_grid_topic_arg = DeclareLaunchArgument('unknown_grid_topic',   default_value='unknown_grid',                description='output unknown grid topic')
    front_grid_topic_arg = DeclareLaunchArgument('frontier_grid_topic',  default_value='frontier_grid',               description='output frontier grid topic')
    dist_grid_topic_arg = DeclareLaunchArgument('distance_grid_topic',  default_value='distance_grid',               description='output distance grid topic')
    cost_grid_topic_arg = DeclareLaunchArgument('cost_grid_topic',      default_value='cost_grid',                   description='output cost grid topic')
    path_topic_arg = DeclareLaunchArgument('path_topic',           default_value='path',                        description='output path topic')
    dyn_grid_topic_arg = DeclareLaunchArgument('dynamic_grid_topic',           default_value='dynamic_grid',                        description='output dynamic grid topic')
    sparse_path_topic_arg = DeclareLaunchArgument('sparse_path_topic',    default_value='sparse_path',                 description='output sparse path topic')
    init_x_arg = DeclareLaunchArgument('init_x',    default_value='0.0',                 description='vehicle initial pose')
    init_y_arg = DeclareLaunchArgument('init_y',    default_value='0.0',                 description='vehicle initial pose')
    init_z_arg = DeclareLaunchArgument('init_z',    default_value='0.0',                 description='vehicle initial pose')
    init_yaw_arg = DeclareLaunchArgument('init_yaw',    default_value='0.0',                 description='vehicle initial pose')
    init_pitch_arg = DeclareLaunchArgument('init_pitch',    default_value='0.0',                 description='vehicle initial pose')
    init_roll_arg = DeclareLaunchArgument('init_roll',    default_value='0.0',                 description='vehicle initial pose')


    # Opaque function to launch nodes
    def launch_setup(context, *args, **kwargs):

        # get launch arguments  
        namespace              = LaunchConfiguration('namespace').perform(context)
        use_tracker            = convert_str_to_bool(LaunchConfiguration('use_tracker').perform(context))
        publish_init_tf            = convert_str_to_bool(LaunchConfiguration('publish_init_tf').perform(context))
        param_file             = LaunchConfiguration('param_file').perform(context)
        depth_pointcloud_topic = LaunchConfiguration('depth_pointcloud_topic').perform(context)
        pose_topic             = LaunchConfiguration('pose_topic').perform(context)
        goal_topic             = LaunchConfiguration('goal_topic').perform(context)
        odom_topic             = LaunchConfiguration('odom_topic').perform(context)
        occupancy_grid_topic   = LaunchConfiguration('occupancy_grid_topic').perform(context)
        unknown_grid_topic     = LaunchConfiguration('unknown_grid_topic').perform(context)
        frontier_grid_topic    = LaunchConfiguration('frontier_grid_topic').perform(context)
        distance_grid_topic    = LaunchConfiguration('distance_grid_topic').perform(context)
        cost_grid_topic        = LaunchConfiguration('cost_grid_topic').perform(context)
        path_topic             = LaunchConfiguration('path_topic').perform(context)
        dynamic_grid_topic     = LaunchConfiguration('dynamic_grid_topic').perform(context)
        sparse_path_topic      = LaunchConfiguration('sparse_path_topic').perform(context)
        init_x                 = LaunchConfiguration('init_x').perform(context)
        init_y                 = LaunchConfiguration('init_y').perform(context)
        init_z                 = LaunchConfiguration('init_z').perform(context)
        init_yaw               = LaunchConfiguration('init_yaw').perform(context)
        init_pitch             = LaunchConfiguration('init_pitch').perform(context)
        init_roll              = LaunchConfiguration('init_roll').perform(context)

        # The path to the parameter file
        parameters_path=os.path.join(get_package_share_directory('global_mapper_ros'), 'cfg', param_file)

        # Get the dict of parameters from the yaml file
        with open(parameters_path, 'r') as file:
            parameters = yaml.safe_load(file)

        # Extract specific node parameters
        parameters = parameters['flight_goggles']['ros__parameters']
        
        # Create a global mapper node 
        global_mapper_node = Node(
            package='global_mapper_ros',
            executable='global_mapper_node',
            name='global_mapper_ros',
            namespace=namespace,
            output='screen',
            parameters=[parameters],
            remappings=[
                ('depth_pointcloud_topic', depth_pointcloud_topic),
                ('pose_topic',           pose_topic),
                ('goal_topic',           goal_topic),
                ('odom_topic',           odom_topic),
                ('occupancy_grid_topic', occupancy_grid_topic),
                ('unknown_grid_topic',   unknown_grid_topic),
                ('frontier_grid_topic',  frontier_grid_topic),
                ('distance_grid_topic',  distance_grid_topic),
                ('cost_grid_topic',      cost_grid_topic),
                ('path_topic',           path_topic),
                ('dynamic_grid_topic',   dynamic_grid_topic),
                ('sparse_path_topic',    sparse_path_topic),
            ],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
        )
    
        # Create an obstacle tracker node
        obstacle_tracker_node = Node(
            package='global_mapper_ros',
            executable='obstacle_tracker_node',
            namespace=namespace,
            name='obstacle_tracker_node',
            emulate_tty=True,
            parameters=[parameters],
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            output='screen',
            remappings=[('point_cloud', f'dynamic_grid')],
        )

        init_pose_tf = Node( 
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='init_pose_to_world_mocap',
            arguments=[init_x, init_y, init_z, init_yaw, init_pitch, init_roll, "world_mocap", f"{namespace}/init_pose"]
        )

        nodes_to_start = [
                        global_mapper_node
                        ]

        if publish_init_tf: 
            nodes_to_start = [init_pose_tf] + nodes_to_start 
            
        if use_tracker: 
            nodes_to_start.append(obstacle_tracker_node)

        return nodes_to_start

    # Create launch description
    return LaunchDescription([
        namespace_arg,
        use_tracker_arg,
        publish_init_tf_arg,
        param_file_arg,
        pc_arg,
        pose_topic_arg,
        goal_topic_arg,
        odom_topic_arg,
        occ_grid_topic_arg,
        unk_grid_topic_arg,
        front_grid_topic_arg,
        dist_grid_topic_arg,
        cost_grid_topic_arg,
        path_topic_arg,
        dyn_grid_topic_arg, 
        sparse_path_topic_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_yaw_arg,
        init_pitch_arg,
        init_roll_arg,
        OpaqueFunction(function=launch_setup)
    ])