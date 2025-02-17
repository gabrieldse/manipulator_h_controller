import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    use_dummy_arg = DeclareLaunchArgument("use_dummy", default_value="true", description="Use dummy hardware")
    use_dummy = LaunchConfiguration("use_dummy")
    
    
    moveit_config = (
        MoveItConfigsBuilder("manipulator_h", package_name="manipulator_h_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("manipulator_h_description"),"urdf","manipulator_h_ros2_control.urdf.xacro"),
            mappings={"use_dummy": use_dummy})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("manipulator_h_moveit_config") + "/config/move_group.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link1"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("manipulator_h_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    
    load_controllers = []
    for controller in [
        "manipulator_h_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers.append(
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner", controller],
                output="screen",
            )
        )

    return LaunchDescription(
        [
            rviz_node,
            use_dummy_arg,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ] + load_controllers
    )

