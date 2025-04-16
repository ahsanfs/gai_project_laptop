import struct
import socket

def trigger_screwdriver_cw():
    ip = "172.16.0.1"    
    port = 5000
    header = 0xA1
    cmd = 0x06   
    payload = b'\x01\x00'   

    def calculate_crc(data):
        crc = 0x5A
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    payload_size = len(payload)
    packet = struct.pack('>B B H', header, cmd, payload_size) + payload
    crc = calculate_crc(packet)
    packet += struct.pack('>B', crc)
    print("Packet to send:", packet.hex())
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((ip, port))
            sock.sendall(packet)
            print("Packet sent successfully.")
    except socket.error as e:
        print("Socket error:", e)

if __name__ == "__main__":
    trigger_screwdriver_cw()
