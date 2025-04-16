import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import socket
import struct


class LockScrewService(Node):
    def __init__(self):
        super().__init__('lock_screw_service')
        self.srv = self.create_service(Trigger, 'lock_screw', self.handle_lock_screw)
        self.get_logger().info('Lock Screw Service is ready.')

    def calculate_crc(self, data):
        """Calculate CRC for packet."""
        crc = 0x5A
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def handle_lock_screw(self, request, response):
        """Handle lock_screw service request."""
        ip = "172.16.0.1"
        port = 5000
        header = 0xA1
        cmd = 0x06
        payload = b'\x01\x00'
        payload_size = len(payload)

        packet = struct.pack('>B B H', header, cmd, payload_size) + payload
        crc = self.calculate_crc(packet)
        packet += struct.pack('>B', crc)

        try:
            self.get_logger().info(f"Sending packet to {ip}:{port} -> {packet.hex()}")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((ip, port))
                sock.sendall(packet)
                self.get_logger().info("Packet sent successfully.")
            response.success = True
            response.message = "Screw locked successfully."
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")
            response.success = False
            response.message = f"Failed to lock screw: {e}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LockScrewService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
