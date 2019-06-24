# Launch file for agent node

import launch
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    id_ = launch.substitutions.LaunchConfiguration('id')
    agent_name = launch.substitutions.LaunchConfiguration('agent_name')
    log_level = launch.substitutions.LaunchConfiguration('log_level')
    sensors = launch.substitutions.LaunchConfiguration('sensors')
    planner = launch.substitutions.LaunchConfiguration('planner')

    return launch.LaunchDescription([

        # create arguments
        launch.actions.DeclareLaunchArgument(
            'id',
            description='agent id (int)'
        ),
        
        launch.actions.DeclareLaunchArgument(
            'agent_name',
            # default_value=
        ),
        
        launch.actions.DeclareLaunchArgument(
            'log_level',
            default_value='INFO'
        ),
        
        launch.actions.DeclareLaunchArgument(
            'planner',
            default_value='false'
        ),
        
        launch.actions.DeclareLaunchArgument(
            'sensors',
            default_value='false'
        ),

        ### launch ET-DDF nodes

        # agent wrapper
        launch_ros.actions.Node(
            package='etddf_ros2',
            node_executable='agent_wrapper',
            output='screen',
            # parameters=['ros_agent_config.yaml']
            arguments=[id_,agent_name,log_level]
        ),

        # comms module
        launch_ros.actions.Node(
            package='etddf_ros2',
            node_executable='comms',
            output='screen',
            # parameters=[get_package_share_directory('etddf_ros2') + '/ros_agent_config.yaml']
            arguments=[agent_name],
        ),

        ### launch simulation nodes

        # if launch.substitutions.LaunchConfiguration('planner'):
        launch_ros.actions.Node(
            package='etddf_ros2',
            node_executable='point_planner',
            output='screen',
            arguments=[agent_name]
        ),

        # if sensors:
        launch_ros.actions.Node(
            package='etddf_ros2',
            node_executable='sensors',
            output='screen',
            arguments=[agent_name]
        )

    ])