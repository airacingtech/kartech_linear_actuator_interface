from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    # get package directory
    pkg_dir = get_package_share_directory("kartech_linear_actuator_interface_can")

    dbc_file_path = DeclareLaunchArgument(
        "dbc_file_path",
        default_value=os.path.join(pkg_dir, "config", "Kartech_Linear_Actuator.dbc"),
    )

    kartech_linear_actuator_interface_param_file = os.path.join(
        pkg_dir, "param", "kartech_linear_actuator_interface.param.yaml"
    )

    socket_can_receiver_node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        namespace=TextSubstitution(text=""),
        parameters=[
            {
                "interface": LaunchConfiguration("interface"),
                "interval_sec": LaunchConfiguration("interval_sec"),
                "use_bus_time": LaunchConfiguration("use_bus_time"),
            }
        ],
        output="screen",
    )

    socket_can_receiver_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_configure")),
    )

    socket_can_receiver_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_activate")),
    )

    socket_can_sender_node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_sender_node_exe",
        name="socket_can_sender",
        namespace=TextSubstitution(text=""),
        parameters=[
            {
                "interface": LaunchConfiguration("interface"),
                "timeout_sec": LaunchConfiguration("timeout_sec"),
            }
        ],
        output="screen",
    )

    socket_can_sender_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_sender_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_configure")),
    )

    socket_can_sender_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_sender_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_activate")),
    )

    kartech_linear_actuator_interface_node = Node(
        package="kartech_linear_actuator_interface_can",
        executable="kartech_linear_actuator_interface_can_node",
        output="screen",
        namespace="kartech_linear_actuator_interface_interface",
        parameters=[
            kartech_linear_actuator_interface_param_file,
            {"dbc_file": LaunchConfiguration("dbc_file_path")},
        ],
        remappings=[
            ("can_tx", "/from_can_bus"),
            ("can_rx", "/to_can_bus"),
        ],
    )

    launch_description = [
        DeclareLaunchArgument("interface", default_value="can0"),
        DeclareLaunchArgument("interval_sec", default_value="0.01"),
        DeclareLaunchArgument("auto_configure", default_value="true"),
        DeclareLaunchArgument("auto_activate", default_value="true"),
        DeclareLaunchArgument("use_bus_time", default_value="false"),
        socket_can_receiver_node,
        socket_can_receiver_configure_event_handler,
        socket_can_receiver_activate_event_handler,
        DeclareLaunchArgument("interface", default_value="can0"),
        DeclareLaunchArgument("timeout_sec", default_value="0.01"),
        DeclareLaunchArgument("auto_configure", default_value="true"),
        DeclareLaunchArgument("auto_activate", default_value="true"),
        socket_can_sender_node,
        socket_can_sender_configure_event_handler,
        socket_can_sender_activate_event_handler,
        kartech_linear_actuator_interface_node,
        dbc_file_path,
    ]

    return LaunchDescription(launch_description)
