#!/usr/bin/env python3

"""
Test script for Hesai LiDAR raw data system
Simulates UDP packets to test the driver without hardware
"""

import socket
import struct
import time
import threading
import argparse
import math

class HesaiPacketSimulator:
    """Simulates Hesai Pandar 40p UDP packets for testing"""
    
    def __init__(self, target_ip="127.0.0.1", data_port=2368, position_port=8308):
        self.target_ip = target_ip
        self.data_port = data_port
        self.position_port = position_port
        self.running = False
        
        # Packet structure constants
        self.PACKET_SIZE = 1262
        self.HEADER_SIZE = 42
        self.BLOCKS_PER_PACKET = 10
        self.CHANNELS_PER_BLOCK = 40
        
        # Create sockets
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.position_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print(f"Hesai Packet Simulator configured:")
        print(f"  Target IP: {self.target_ip}")
        print(f"  Data Port: {self.data_port}")
        print(f"  Position Port: {self.position_port}")
    
    def create_data_packet(self, azimuth_start=0.0):
        """Create a simulated Hesai data packet"""
        packet = bytearray(self.PACKET_SIZE)
        
        # Header (simplified)
        packet[0] = 0xEE  # Magic byte 1
        packet[1] = 0xFF  # Magic byte 2
        
        # Fill remaining header with zeros for simplicity
        header_data = struct.pack('<H', 0x4050)  # Product ID (Pandar 40p)
        packet[2:4] = header_data
        
        # Data blocks
        offset = self.HEADER_SIZE
        
        for block in range(self.BLOCKS_PER_PACKET):
            # Calculate azimuth for this block (rotating)
            azimuth = (azimuth_start + block * 3.6) % 360.0  # 3.6 degrees per block
            azimuth_raw = int(azimuth * 100)  # Convert to centidegrees
            
            # Block header: azimuth (2 bytes) + flag (2 bytes)
            block_header = struct.pack('<HH', azimuth_raw, 0xEEFF)
            packet[offset:offset+4] = block_header
            offset += 4
            
            # Channel data (40 channels × 3 bytes each)
            for channel in range(self.CHANNELS_PER_BLOCK):
                # Simulate distance based on channel and azimuth
                base_distance = 10.0 + channel * 0.5  # Vary by channel
                distance_variation = 2.0 * math.sin(math.radians(azimuth + channel * 9))
                distance = base_distance + distance_variation
                
                # Convert to raw format (4mm resolution)
                distance_raw = int(distance / 0.004)
                distance_raw = max(0, min(65535, distance_raw))  # Clamp to uint16
                
                # Simulate intensity
                intensity = 100 + int(50 * math.sin(math.radians(azimuth * 2 + channel * 5)))
                intensity = max(0, min(255, intensity))  # Clamp to uint8
                
                # Pack channel data: distance (2 bytes) + intensity (1 byte)
                channel_data = struct.pack('<HB', distance_raw, intensity)
                packet[offset:offset+3] = channel_data
                offset += 3
            
            # Skip to next block (account for any padding)
            # Each block should be 130 bytes total
            block_end = self.HEADER_SIZE + (block + 1) * 130
            offset = block_end
        
        return bytes(packet)
    
    def create_position_packet(self):
        """Create a simulated position packet"""
        # Simplified position packet
        packet = bytearray(512)
        
        # Header
        packet[0] = 0xAA
        packet[1] = 0xBB
        
        # Simulated GPS coordinates (latitude, longitude, altitude)
        lat = 51.4545 + 0.0001 * math.sin(time.time() * 0.1)  # Bristol, UK area
        lon = -2.5879 + 0.0001 * math.cos(time.time() * 0.1)
        alt = 50.0 + 5.0 * math.sin(time.time() * 0.05)
        
        # Pack GPS data (simplified format)
        gps_data = struct.pack('<ddd', lat, lon, alt)
        packet[4:28] = gps_data
        
        # Timestamp
        timestamp = int(time.time() * 1000000)  # Microseconds
        ts_data = struct.pack('<Q', timestamp)
        packet[28:36] = ts_data
        
        return bytes(packet[:100])  # Truncate to reasonable size
    
    def send_data_packets(self, rate_hz=10.0):
        """Send data packets at specified rate"""
        interval = 1.0 / rate_hz
        azimuth = 0.0
        packet_count = 0
        
        print(f"Starting data packet transmission at {rate_hz} Hz...")
        
        while self.running:
            try:
                packet = self.create_data_packet(azimuth)
                self.data_socket.sendto(packet, (self.target_ip, self.data_port))
                
                azimuth = (azimuth + 36.0) % 360.0  # Full rotation per 10 packets
                packet_count += 1
                
                if packet_count % 100 == 0:
                    print(f"Sent {packet_count} data packets")
                
                time.sleep(interval)
                
            except Exception as e:
                print(f"Error sending data packet: {e}")
                break
    
    def send_position_packets(self, rate_hz=1.0):
        """Send position packets at specified rate"""
        interval = 1.0 / rate_hz
        packet_count = 0
        
        print(f"Starting position packet transmission at {rate_hz} Hz...")
        
        while self.running:
            try:
                packet = self.create_position_packet()
                self.position_socket.sendto(packet, (self.target_ip, self.position_port))
                
                packet_count += 1
                
                if packet_count % 10 == 0:
                    print(f"Sent {packet_count} position packets")
                
                time.sleep(interval)
                
            except Exception as e:
                print(f"Error sending position packet: {e}")
                break
    
    def start(self, data_rate=10.0, position_rate=1.0):
        """Start packet transmission"""
        self.running = True
        
        # Start data packet thread
        self.data_thread = threading.Thread(
            target=self.send_data_packets, 
            args=(data_rate,))
        self.data_thread.daemon = True
        self.data_thread.start()
        
        # Start position packet thread
        self.position_thread = threading.Thread(
            target=self.send_position_packets, 
            args=(position_rate,))
        self.position_thread.daemon = True
        self.position_thread.start()
        
        print("Packet simulator started. Press Ctrl+C to stop.")
    
    def stop(self):
        """Stop packet transmission"""
        print("Stopping packet simulator...")
        self.running = False
        
        if hasattr(self, 'data_thread'):
            self.data_thread.join(timeout=1.0)
        if hasattr(self, 'position_thread'):
            self.position_thread.join(timeout=1.0)
        
        self.data_socket.close()
        self.position_socket.close()
        print("Packet simulator stopped.")

def main():
    parser = argparse.ArgumentParser(description='Hesai LiDAR Packet Simulator')
    parser.add_argument('--ip', default='127.0.0.1', help='Target IP address')
    parser.add_argument('--data-port', type=int, default=2368, help='Data port')
    parser.add_argument('--position-port', type=int, default=8308, help='Position port')
    parser.add_argument('--data-rate', type=float, default=10.0, help='Data packet rate (Hz)')
    parser.add_argument('--position-rate', type=float, default=1.0, help='Position packet rate (Hz)')
    parser.add_argument('--duration', type=float, default=0, help='Duration to run (0 = infinite)')
    
    args = parser.parse_args()
    
    # Create simulator
    simulator = HesaiPacketSimulator(args.ip, args.data_port, args.position_port)
    
    try:
        # Start simulation
        simulator.start(args.data_rate, args.position_rate)
        
        # Run for specified duration or until interrupted
        if args.duration > 0:
            time.sleep(args.duration)
            simulator.stop()
        else:
            # Run until Ctrl+C
            while True:
                time.sleep(1)
                
    except KeyboardInterrupt:
        simulator.stop()

if __name__ == '__main__':
    main()
