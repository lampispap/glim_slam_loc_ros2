from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    use_sim_time = LaunchConfiguration("use_sim_time")

    config_launch_arg = DeclareLaunchArgument(
        "config", default_value=TextSubstitution(text="config/b2_simulation")
    )

    localization_launch_arg = DeclareLaunchArgument(
        "localization", default_value="false"
    )

    map_path_launch_arg = DeclareLaunchArgument(
        "map_path", default_value="/home/manh/Documents/park_gps"
    )

    # TODO: Add it with relative path
    saved_map_path_launch_arg = DeclareLaunchArgument(
        "map_save_path",
        default_value="/home/lampis/study_ws/glim_loc_ws/test_maps/only_test",
    )

    glim_ros_node = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_ros",
        parameters=[
            {
                "config_path": LaunchConfiguration("config"),
                "localization": LaunchConfiguration("localization"),
                "map_path": LaunchConfiguration("map_path"),
                "map_save_path": LaunchConfiguration("map_save_path"),
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
        # prefix=["gnome-terminal -- gdb -ex run --args"]
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("ROS_DOMAIN_ID", "42"))
    ld.add_action(SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"))
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_launch_arg)
    ld.add_action(localization_launch_arg)
    ld.add_action(map_path_launch_arg)
    ld.add_action(saved_map_path_launch_arg)
    ld.add_action(glim_ros_node)

    # Add the commands to the launch description

    return ld
