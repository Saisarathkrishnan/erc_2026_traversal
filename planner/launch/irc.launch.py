from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # YOLO (VISIBLE)
    inference_engine = Node(
        package='yolo',
        executable='inference_engine',
        name='inference_engine',
        output='screen'
    )

    # ESP bridge (VISIBLE)
    relay_node = Node(
        package='planner',
        executable='relay_node',
        name='relay_node',
        output='screen'
    )

    # ZED2i camera (SILENT)
    zed_camera = ExecuteProcess(
        cmd=[
            'bash','-c',
            "ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i' > /dev/null 2>&1"
        ]
    )

    # Local obstacle filter (SILENT)
    local_obstacle_filter = ExecuteProcess(
        cmd=[
            'bash','-c',
            'ros2 run planner local_obstacle_filter_node > /dev/null 2>&1'
        ]
    )

    # RTK GPS (SILENT)
    gps_rtk = ExecuteProcess(
        cmd=[
            'bash','-c',
            'ros2 run gps gps_rtk > /dev/null 2>&1'
        ]
    )

    # ex_imu (SILENT)
    ex_imu = ExecuteProcess(
        cmd=[
            'bash','-c',
            'ros2 run ex_imu ex_imu > /dev/null 2>&1'
        ]
    )

    # imu_conversion (SILENT)
    imu_conv = ExecuteProcess(
        cmd=[
            'bash','-c',
            'ros2 run planner imu_conversion_node > /dev/null 2>&1'
        ]
    )

    return LaunchDescription([
        zed_camera,
        inference_engine,
        relay_node,
        local_obstacle_filter,
        gps_rtk,
        ex_imu,
        imu_conv
    ])
