from launch import LaunchDescription
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

    return LaunchDescription([

        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id": "zed_camera_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "approx_sync": True,
                "sync_queue_size": 30,
                "wait_for_transform": 0.5,
                "rate": 10,
                "Odom/ResetCountdown": "1",
                "Odom/Strategy": "0",
                "Odom/MaxFeatures": "1000",
                "Odom/MinInliers": "15",
                "Vis/MinInliers": "15",
                "Vis/InlierDistance": "0.1",
                "Vis/EstimationType": "0"
            }],
            remappings=remaps
        ),

        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[{
                "frame_id": "zed_camera_link",
                "odom_frame_id": "odom",
                "map_frame_id": "map",
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_odom_info": True,
                "approx_sync": True,
                "sync_queue_size": 30,
                "wait_for_transform": 0.5,
                "rate": 10,
                "Mem/IncrementalMemory": "true",
                "Mem/InitWMWithAllNodes": "false",
                "RGBD/OptimizeMaxError": "3.0",
                "Grid/FromDepth": "true",
                "Grid/RangeMax": "10.0",
                "Grid/CellSize": "0.05",
                "Grid/UseRayTracing": "true",
                "Grid/FlatObstacleDetected": "true",
                "Grid/GroundIsObstacle": "false",
                "Grid/MaxGroundAngle": "15.0",
                "Grid/MaxObstacleHeight": "1.5",
                "Grid/MinGroundHeight": "0.20"
            }],
            remappings=remaps
        ),

        Node(
            package="rtabmap_util",
            executable="point_cloud_xyzrgb",
            name="point_cloud_xyzrgb",
            output="screen",
            parameters=[{
                "approx_sync": True,
                "sync_queue_size": 30,
                "wait_for_transform": 0.5
            }],
            remappings=remaps
        ),
    ])
