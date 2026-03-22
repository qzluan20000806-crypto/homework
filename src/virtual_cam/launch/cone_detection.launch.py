from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO'
    )
    log_level = LaunchConfiguration('log_level')

    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='cone_detection_container',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    virtual_cam_node = LoadComposableNodes(
        target_container='cone_detection_container',
        composable_node_descriptions=[
            ComposableNode(
                package='virtual_cam',
                plugin='VirtualCamNode', 
                name='virtual_cam_component',
                extra_arguments=[{'use_intra_process_comms': True}]  # 进程内通信（零拷贝优化）
            )
        ]
    )

    load_cone_detector = LoadComposableNodes(
        target_container='cone_detection_container',
        composable_node_descriptions=[
            ComposableNode(
                package='virtual_cam',
                plugin='ImageSubscriberComponent', 
                name='cone_detector_component',
                extra_arguments=[{'use_intra_process_comms': True}]  # 进程内通信（零拷贝优化）
            )
        ]
    )
    return LaunchDescription([
        log_level_arg,
        container,
        virtual_cam_node,
        load_cone_detector
    ])