import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    package_name = "manipulator_h_description"
    manipulator_description_path = os.path.join(
        get_package_share_directory('manipulator_h_description'), 'urdf', 'manipulator_h_with_umi_gripper.xacro'
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory('manipulator_h_description'), 'launch', 'manipulator_h.rviz'
    )

    robot_description_config = xacro.process_file(manipulator_description_path)
     
    return LaunchDescription([    
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),
    

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
        
        # GUI state publisher
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"),
    ])
