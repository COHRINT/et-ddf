# Launch file for agent node

import launch
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # id_ = launch.substitutions.LaunchConfiguration('id')
    # agent_name = launch.substitutions.LaunchConfiguration('agent_name')
    # log_level = launch.substitutions.LaunchConfiguration('log_level')
    # sensors = launch.substitutions.LaunchConfiguration('sensors')
    # planner = launch.substitutions.LaunchConfiguration('planner')

    return launch.LaunchDescription([

        # create arguments
        # launch.actions.DeclareLaunchArgument(
        #     'id',
        #     description='agent id (int)'
        # ),
        
        # launch.actions.DeclareLaunchArgument(
        #     'agent_name',
        #     # default_value=
        # ),
        
        # launch.actions.DeclareLaunchArgument(
        #     'log_level',
        #     default_value='INFO'
        # ),
        
        # launch.actions.DeclareLaunchArgument(
        #     'planner',
        #     default_value='false'
        # ),
        
        # launch.actions.DeclareLaunchArgument(
        #     'sensors',
        #     default_value='false'
        # ),

        ### launch ET-DDF nodes

        launch_ros.actions.Node(
            package='etddf_ros2',
            node_executable='point_sim',
            output='screen',
        ),

        # agent wrapper
       launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([launch.substitutions.ThisLaunchFileDir(), '/agent.launch.py']),
            launch_arguments={'agent_name': 'agent_0',
                                'id': '0',
                                'planner': 'True',
                                'sensors': 'True',
                                'log_level': 'INFO'}.items(),
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([launch.substitutions.ThisLaunchFileDir(), '/agent.launch.py']),
            launch_arguments={'agent_name': 'agent_1',
                                'id': '1',
                                'planner': 'True',
                                'sensors': 'True'}.items(),
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([launch.substitutions.ThisLaunchFileDir(), '/agent.launch.py']),
            launch_arguments={'agent_name': 'agent_2',
                                'id': '2',
                                'planner': 'True',
                                'sensors': 'True'}.items(),
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([launch.substitutions.ThisLaunchFileDir(), '/agent.launch.py']),
            launch_arguments={'agent_name': 'agent_3',
                                'id': '3',
                                'planner': 'True',
                                'sensors': 'True'}.items(),
        )

    ])