from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("uss_nav_bringup"), "config", "params.yaml"]
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to USS-Nav parameter file.",
    )
    use_vlm_arg = DeclareLaunchArgument(
        "use_vlm",
        default_value="false",
        description="Enable VLM visual search node (requires vLLM service).",
    )
    vlm_image_topic_arg = DeclareLaunchArgument(
        "vlm_image_topic",
        default_value="/camera/image/compressed",
        description="Image topic consumed by VLM search node.",
    )
    vlm_image_is_compressed_arg = DeclareLaunchArgument(
        "vlm_image_is_compressed",
        default_value="true",
        description="Whether VLM image topic is sensor_msgs/CompressedImage.",
    )
    vlm_target_text_arg = DeclareLaunchArgument(
        "vlm_target_text",
        default_value="chair",
        description="Target text used by VLM search node.",
    )
    params_file = LaunchConfiguration("params_file")
    use_vlm = LaunchConfiguration("use_vlm")
    vlm_image_topic = LaunchConfiguration("vlm_image_topic")
    vlm_image_is_compressed = LaunchConfiguration("vlm_image_is_compressed")
    vlm_target_text = LaunchConfiguration("vlm_target_text")

    nodes = [
        Node(package="uss_nav_stack", executable="rolling_grid_node", name="rolling_grid_node", parameters=[params_file]),
        Node(package="uss_nav_stack", executable="gcm_node", name="gcm_node", parameters=[params_file]),
        Node(package="uss_nav_stack", executable="frontier_node", name="frontier_node", parameters=[params_file]),
        Node(package="uss_nav_stack", executable="scg_node", name="scg_node", parameters=[params_file]),
        Node(package="uss_nav_stack", executable="region_node", name="region_node", parameters=[params_file]),
        Node(package="uss_nav_stack", executable="object_graph_node", name="object_graph_node", parameters=[params_file]),
        Node(package="uss_nav_stack", executable="planner_node", name="planner_node", parameters=[params_file]),
        Node(
            package="uss_nav_stack",
            executable="vlm_search_node",
            name="vlm_search_node",
            parameters=[
                params_file,
                {
                    "enabled": use_vlm,
                    "image_topic": vlm_image_topic,
                    "image_is_compressed": vlm_image_is_compressed,
                    "target_text": vlm_target_text,
                },
            ],
            condition=IfCondition(use_vlm),
        ),
    ]

    return LaunchDescription(
        [params_arg, use_vlm_arg, vlm_image_topic_arg, vlm_image_is_compressed_arg, vlm_target_text_arg]
        + nodes
    )
