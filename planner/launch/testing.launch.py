from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    remaps = [
        ("rgb/image", "/zed/zed_node/rgb/color/rect/image"),
        ("rgb/camera_info", "/zed/zed_node/rgb/color/rect/image/camera_info"),
        ("depth/image", "/zed/zed_node/depth/depth_registered"),
        ("depth/camera_info", "/zed/zed_node/depth/depth_registered/camera_info"),
        ("imu", "/zed/zed_node/imu/data"),
        ("odom", "/odom")
    ]



    relay_node = Node(
        package='planner',
        executable='relay_node',
        name='relay_node',
        output='screen'
    )

    zed_camera = ExecuteProcess(
        cmd=[
            'bash','-c',
            "ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i' > /dev/null 2>&1"
        ]
    )
    gps_rtk = ExecuteProcess(
        cmd=[
            'bash','-c',
            'ros2 run gps gps_rtk > /dev/null 2>&1'
        ]
    )

    ex_imu = Node(
        package='ex_imu',
        executable='ex_imu',
        name='ex_imu',
        output='screen'
    )


    return LaunchDescription([
        zed_camera,
        relay_node,
        gps_rtk,
        ex_imu
    ])
