import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

import serial
import pynmea2

PORT = "/dev/ttyACM0"
BAUD = 9600


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            self.get_logger().info(f"Connected to GPS on {PORT}")
        except serial.SerialException:
            self.get_logger().error(f"Failed to open {PORT}")
            exit(1)

        self.timer = self.create_timer(0.1, self.read_gps)  # 10 Hz

    def read_gps(self):
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()

            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                msg = pynmea2.parse(line)

                if msg.gps_qual == 0:
                    return  # no fix

                navsat = NavSatFix()

                # Header
                navsat.header.stamp = self.get_clock().now().to_msg()
                navsat.header.frame_id = "gps"

                # Status
                navsat.status.status = NavSatStatus.STATUS_FIX
                navsat.status.service = NavSatStatus.SERVICE_GPS

                # Position
                navsat.latitude = msg.latitude
                navsat.longitude = msg.longitude
                navsat.altitude = float(msg.altitude) if msg.altitude else 0.0
                fix_quality = msg.gps_qual
                hdop = msg.horizontal_dil 
                # Optional covariance (rough estimate using HDOP)
                if msg.horizontal_dil:
                    hdop = float(msg.horizontal_dil)
                    cov = hdop ** 2
                else:
                    cov = 0.0

                navsat.position_covariance = [
                    cov, 0.0, 0.0,
                    0.0, cov, 0.0,
                    0.0, 0.0, cov * 2
                ]
                navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                self.publisher_.publish(navsat)

                self.get_logger().info(
                    f"Lat: {navsat.latitude:.6f}, Lon: {navsat.longitude:.6f}, FIX: {fix_quality}, HDOP: {hdop:.3f}"
                )

        except pynmea2.ParseError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()