from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur20_display"),  # also typo: "u20_display"
            "urdf",
            "ur20_with_gripper.urdf.xacro"
        ])
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
    )
    

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher,
        rviz_node
    ])
