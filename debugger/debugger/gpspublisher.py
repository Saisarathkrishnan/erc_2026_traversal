import rclpy
from rclpy.node import Node

from sensor_msgs.msg import  NavSatFix
from custom_msgs.msg   import ImuData

import tkinter as tk
from threading import Thread
import time


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_publisher')

        # Publishers
        self.imu_pub = self.create_publisher(ImuData, '/imu_data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/fix', 10)

        # Data storage
        self.angle = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        # Timer to publish data
        self.create_timer(0.1, self.publish_data)


    def publish_data(self):
        # Publish IMU angle
        imu_msg = ImuData()
        imu_msg.orientation.z = float(self.angle)
        self.imu_pub.publish(imu_msg)

        # Publish GPS data
        gps_msg = NavSatFix()
        gps_msg.latitude = float(self.lat)
        gps_msg.longitude = float(self.lon)
        gps_msg.altitude = float(self.alt)

        self.gps_pub.publish(gps_msg)


class GuiApp:
    def __init__(self, ros_node):
        self.node = ros_node

        self.root = tk.Tk()
        self.root.title("gps and imu input for debugging")

        # Slider for angle
        self.slider = tk.Scale(
            self.root,
            from_=0,
            to=360,
            orient=tk.HORIZONTAL,
            label="Angle (degrees)",
            command=self.update_angle,
            length=400
        )
        self.slider.pack()

        # Latitude input
        tk.Label(self.root, text="Latitude").pack()
        self.lat_entry = tk.Entry(self.root)
        self.lat_entry.pack()

        # Longitude input
        tk.Label(self.root, text="Longitude").pack()
        self.lon_entry = tk.Entry(self.root)
        self.lon_entry.pack()

        # Altitude input
        tk.Label(self.root, text="Altitude").pack()
        self.alt_entry = tk.Entry(self.root)
        self.alt_entry.pack()

        # Update button
        tk.Button(self.root, text="Update GPS", command=self.update_gps).pack()

    def update_angle(self, val):
        self.node.angle = float(val)

    def update_gps(self):
        try:
            self.node.lat = float(self.lat_entry.get())
            self.node.lon = float(self.lon_entry.get())
            self.node.alt = float(self.alt_entry.get())
        except ValueError:
            print("Invalid gps ")

    def run(self):
        self.root.mainloop()


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    node = GuiNode()

    # Run ROS in separate thread
    ros_thread = Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run GUI
    app = GuiApp(node)
    app.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()