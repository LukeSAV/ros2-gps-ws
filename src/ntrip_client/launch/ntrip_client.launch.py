import launch
import launch.actions
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_executable='ntrip_client', 
            package='ntrip_client', 
            #output='screen',
            output={
                'stdout' : 'screen',
                'stderr' : 'screen'
            },
            parameters=['/var/lib/ntrip-config/service.yaml']
        )

    ])