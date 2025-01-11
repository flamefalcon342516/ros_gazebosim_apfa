import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf
import xacro

def generate_launch_description():

    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/test_world3.world'
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()
    print(f"Processing xacro file: {pathModelFile}")

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
        
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items())

    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )
    
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNode)
    ld.add_action(nodeRobotStatePublisher)

    return ld