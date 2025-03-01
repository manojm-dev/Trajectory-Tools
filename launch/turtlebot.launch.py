from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to the nav2_bringup package and your package
    nav2_bringup_share = FindPackageShare('nav2_bringup').find('nav2_bringup')
    trajectory_tools_share = FindPackageShare('trajectory_tools').find('trajectory_tools')
    
    rviz_config_file = os.path.join(trajectory_tools_share, 'rviz', 'display.rviz')
    models_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    
    # Declare the launch configuration variables
    headless = LaunchConfiguration('headless')
    
    declare_launch_arguments = [
        # Declare the headless argument
        DeclareLaunchArgument(
            name='headless',
            default_value='False',
            description='Run in headless mode (no GUI)'
        )
    ]
    
    # Set the TURTLEBOT3_MODEL environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Get the current value of GAZEBO_MODEL_PATH and append the new path
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH']  + ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path 

    # Include the tb3_simulation_launch from the nav2_bringup package
    launch_turtlebot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': headless,
            'rviz_config_file': rviz_config_file
        }.items()
    )

    return LaunchDescription(declare_launch_arguments + [
        launch_turtlebot_sim
    ])
