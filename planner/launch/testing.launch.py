from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():



    inference_engine = Node(
        package='yolo',
        executable='inference_engine',
        name='inference_engine',
        output='screen'
    )

    relay_node = Node(
        package='planner',
        executable='relay_node',
        name='relay_node',
        output='screen'
    )


    local_obstacle_filter_silent = ExecuteProcess(
         cmd=[
             'bash', '-c',
             'ros2 run planner local_obstacle_filter_node > /dev/null 2>&1'
         ]
     )

    gps_rtk_silent = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ros2 run gps gps_rtk > /dev/null 2>&1'
        ]
    )

    return LaunchDescription([
        inference_engine,
        relay_node,
        local_obstacle_filter_silent,
        gps_rtk_silent
    ])
