from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    bag_in_arg = DeclareLaunchArgument(
        'bag_in',
        default_value='',
        description='Input bag file path'
    )
    
    bag_out_arg = DeclareLaunchArgument(
        'bag_out',
        default_value='',
        description='Output bag file path'
    )

    # Launch configurations
    bag_in = LaunchConfiguration('bag_in')
    bag_out = LaunchConfiguration('bag_out')
    
    # Play the input bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_in],
        name='bag_play',
        output='screen'
    )
    
    # Record all topics
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_out],
        name='bag_record',
        output='screen'
    )
    
    # Person detector node
    detector = Node(
        package='project_3',
        executable='person_detector',
        name='person_detector',
        output='screen'
    )
    
    # Person tracker node
    tracker = Node(
        package='project_3',
        executable='person_tracker',
        name='person_tracker',
        output='screen'
    )
    
    # Handler to terminate everything when bag play finishes
    terminate_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bag_play,
            on_exit=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'ros2 bag record'],
                    name='kill_record',
                    output='screen'
                ),
                # Add a small delay to allow for cleanup
                ExecuteProcess(
                    cmd=['sleep', '1'],
                    output='screen'
                ),
                # Emit shutdown event to terminate the launch
                EmitEvent(
                    event=Shutdown(
                        reason='Bag play complete')
                )
            ]
        )
    )
    
    return LaunchDescription([
        bag_in_arg,
        bag_out_arg,
        bag_play,
        bag_record,
        detector,
        tracker,
        terminate_handler
    ])

