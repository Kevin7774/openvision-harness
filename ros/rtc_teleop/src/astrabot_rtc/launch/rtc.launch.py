from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_path = get_package_share_directory("astrabot_rtc") + "/config/rtc.yaml"
    config_argument = DeclareLaunchArgument(
        "rtc_config_path",
        default_value=EnvironmentVariable("ASTRABOT_RTC_CONFIG", default_value=default_config_path),
        description="astrabot_rtc YAML config path",
    )

    rtc_node = Node(
        package="astrabot_rtc",
        executable="astrabot_rtc_node",
        name="astrabot_rtc",
        output="screen",
        parameters=[{"rtc_config_path": LaunchConfiguration("rtc_config_path")}],
    )

    return LaunchDescription([config_argument, rtc_node])
