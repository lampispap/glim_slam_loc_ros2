import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
GLIM_MAPS = os.path.normpath(os.path.join(LAUNCH_DIR, "..", "..", "generated_maps"))


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    config_launch_arg = DeclareLaunchArgument(
        "config", default_value=TextSubstitution(text="config/b2_inspection")
    )

    localization_launch_arg = DeclareLaunchArgument(
        "localization", default_value="false"
    )

    map_path_launch_arg = DeclareLaunchArgument(
        "map_load_path",
        default_value=os.path.join(GLIM_MAPS, "kiefer_office"),
    )

    saved_map_path_launch_arg = DeclareLaunchArgument(
        "map_save_path",
        default_value=os.path.join(GLIM_MAPS, "only_test"),
    )

    base_to_imu_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_static",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "1.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "rslidar",
        ],
        output="screen",
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
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_launch_arg)
    ld.add_action(localization_launch_arg)
    ld.add_action(map_path_launch_arg)
    ld.add_action(saved_map_path_launch_arg)
    ld.add_action(base_to_imu_static)
    ld.add_action(glim_ros_node)
    ld.add_action(hdl_global_loc_node)

    return ld
