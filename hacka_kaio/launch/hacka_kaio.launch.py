from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import RegisterEventHandler, EmitEvent

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg

def generate_launch_description():
#Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'constants_file',
            default_value=PathJoinSubstitution([FindPackageShare('hacka_kaio'),
                                                'params', 'constants.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

#Initialize arguments
    constants_file = LaunchConfiguration('constants_file')

    hacka_kaio_lifecycle_node = LifecycleNode(
        package='hacka_kaio',
        executable='hacka_kaio',
        name='hacka_kaio',
        namespace='',
        output='screen',
        parameters=[constants_file],
        remappings=[('hacka_kaio/have_goal', 'uav1/have_goal'),         #Remap the subscriber: sub_have_goal_
                    ('hacka_kaio/goto', 'uav1/goto'),                   #Remap the publisher: pub_goto_
                    ('hacka_kaio/start_node', 'start_state_machine'),   #Remap the service: srv_start_node
                    ('hacka_kaio/takeoff', 'uav1/takeoff'),             #Remap the client: clt_tackoff_
                    ('hacka_kaio/land', 'uav1/land'),],                 #Remap the client: clt_land_
    )

    event_handlers = []

    event_handlers.append(
#Right after the node starts, make it take the 'configure' transition.
        RegisterEventHandler(
            OnProcessStart(
                target_action=hacka_kaio_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(hacka_kaio_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=hacka_kaio_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(hacka_kaio_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

#Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

#Add client node
    ld.add_action(hacka_kaio_lifecycle_node)

#Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
