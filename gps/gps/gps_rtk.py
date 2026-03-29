import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)

        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        self.timer = self.create_timer(0.1, self.read_gps)

    def convert_to_decimal(self, raw, direction):
        if raw == "":
            return None

        deg_len = 2 if direction in ['N', 'S'] else 3
        degrees = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        decimal = degrees + minutes / 60.0

        if direction in ['S', 'W']:
            decimal *= -1

        return decimal



    def read_gps(self):
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()

            if "$GNGGA" in line or "$GPGGA" in line:
                parts = line.split(',')

                if len(parts) < 10:
                    return

                lat = self.convert_to_decimal(parts[2], parts[3])
                lon = self.convert_to_decimal(parts[4], parts[5])

                fix_quality = int(parts[6]) if parts[6].isdigit() else 0
                altitude = float(parts[9]) if parts[9] else 0.0

                if lat is None or lon is None:
                    return

                # Create ROS message
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "gps"

                # Status mapping
                if fix_quality > 0:
                    msg.status.status = NavSatStatus.STATUS_FIX
                else:
                    msg.status.status = NavSatStatus.STATUS_NO_FIX

                msg.status.service = NavSatStatus.SERVICE_GPS

                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = altitude

                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.publisher_.publish(msg)



                self.get_logger().info(
                    f"{fix_quality} | Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {altitude:.2f}"
                )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


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