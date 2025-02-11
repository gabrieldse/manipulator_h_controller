from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix
import os

os.environ["GAZEBO_MODEL_PATH"] = os.path.join(
    get_package_prefix("manipulator_h_description"),
    "share",
)

print("Package prefix for manipulator_h_description:", get_package_prefix("manipulator_h_description"))

print("GAZEBO_MODEL_PATH:", os.environ["GAZEBO_MODEL_PATH"])

print(FindPackageShare("manipulator_h_description"))
# gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(os.path.join(get_package_prefix("manipulator_h_description")),"share","manipulator_h_description","meshes"))
gazebo_env = SetEnvironmentVariable(
    "GAZEBO_MODEL_PATH",
    os.path.join(
        get_package_prefix("manipulator_h_description"),
        "share",
        "manipulator_h_description",
        "meshes"
    )
)

print("GAZEBO_MODEL_PATH:", os.path.join(
    get_package_prefix("manipulator_h_description"),
    "share",
    "manipulator_h_description",
    "meshes"
))

# Disable the Fuel Model Database
disable_gazebo_model_database = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument("paused", default_value="true", description="Start Gazebo paused"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo with GUI"),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo in headless mode"),
        DeclareLaunchArgument("debug", default_value="true", description="Run Gazebo in debug mode"),
        DeclareLaunchArgument("world", default_value="/root/dev_ws/src/manipulator_h_controller/manipulator_h_gazebo/worlds/empty.world", description="World file to load"),      
    ]

    # Launch configurations
    paused = LaunchConfiguration("paused")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    debug = LaunchConfiguration("debug")
    world = LaunchConfiguration("world")
    
    # Load the URDF into the parameter server
    robot_description_content = Command(
        [
            "xacro ",
            FindPackageShare("manipulator_h_description"),
            "/urdf/manipulator_h.urdf.xacro",
        ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]),
                launch_arguments={"verbose": debug,"world": world}.items(),)

    
    robot_description = {"robot_description": robot_description_content}
    
    
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    

    # Spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'manipulator_h', '-topic', '/robot_description'],
                        output='screen')
    
    
    # return LaunchDescription(
    #     declared_arguments
    #     + [
    #         gazebo,
    #         robot_state_publisher_node,
    #         joint_state_publisher_node,  # Add the joint_state_publisher node
    #         spawn_entity,
    #     ]
    # )
    
    print("GAZEBO_MODEL_PATH:", os.environ.get("GAZEBO_MODEL_PATH"))

    return LaunchDescription(
    [gazebo_env, disable_gazebo_model_database]
    + declared_arguments
    + [
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
    ]
)

