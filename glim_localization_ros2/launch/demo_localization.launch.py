import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
GLIM_MAPS = os.path.normpath(os.path.join(LAUNCH_DIR, "..", "..", "generated_maps"))


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    config_launch_arg = DeclareLaunchArgument(
        "config", default_value=TextSubstitution(text="config/b2_simulation")
    )

    localization_launch_arg = DeclareLaunchArgument(
        "localization", default_value="false"
    )

    map_path_launch_arg = DeclareLaunchArgument(
        "map_load_path",
        default_value=os.path.join(GLIM_MAPS, "gz_playground_dense"),
    )

    saved_map_path_launch_arg = DeclareLaunchArgument(
        "map_save_path",
        default_value=os.path.join(GLIM_MAPS, "only_test"),
    )

    glim_ros_node = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_ros",
        parameters=[
            {
                "config_path": LaunchConfiguration("config"),
                "localization": LaunchConfiguration("localization"),
                "map_load_path": LaunchConfiguration("map_load_path"),
                "map_save_path": LaunchConfiguration("map_save_path"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
        # prefix=["gnome-terminal -- gdb -ex run --args"],
    )

    hdl_global_loc_node = GroupAction(
        condition=IfCondition(LaunchConfiguration("localization")),
        actions=[
            Node(
                package="hdl_global_localization",
                executable="hdl_global_localization_node",
                name="global_localization_ros",
                output="screen",
            )
        ],
    )

    # Add the commands to the launch description
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("ROS_DOMAIN_ID", "42"))
    ld.add_action(SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"))
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_launch_arg)
    ld.add_action(localization_launch_arg)
    ld.add_action(map_path_launch_arg)
    ld.add_action(saved_map_path_launch_arg)
    ld.add_action(glim_ros_node)
    ld.add_action(hdl_global_loc_node)

    return ld
