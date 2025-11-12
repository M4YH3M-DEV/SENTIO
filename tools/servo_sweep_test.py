#!/usr/bin/env python3
"""
Servo Sweep Test Script

Tests servo movement by sending sweep commands to ESP32 and verifying response.

Usage:
  python servo_sweep_test.py --port /dev/ttyUSB0 --servo 0
  python servo_sweep_test.py --port /dev/ttyUSB0 --all
"""

import argparse
import json
import serial
import time
import sys
from typing import Optional


class ServoTester:
    """Test servo controllers via serial."""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        """Initialize servo tester."""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.stats = {
            'commands_sent': 0,
            'responses_received': 0,
            'errors': 0,
            'sweep_success': 0,
        }
    
    def connect(self) -> bool:
        """Connect to ESP32."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for ESP32 boot
            
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            return True
        
        except serial.SerialException as e:
            print(f"✗ Connection failed: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from ESP32."""
        if self.ser:
            self.ser.close()
    
    def send_command(self, command: dict) -> Optional[dict]:
        """Send command and get response."""
        if not self.ser:
            print("✗ Not connected")
            return None
        
        try:
            # Send command
            command_json = json.dumps(command) + '\n'
            self.ser.write(command_json.encode())
            self.stats['commands_sent'] += 1
            
            # Read response
            response_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            if response_line:
                response = json.loads(response_line)
                self.stats['responses_received'] += 1
                return response
            else:
                print("✗ No response received")
                self.stats['errors'] += 1
                return None
        
        except Exception as e:
            print(f"✗ Error: {str(e)}")
            self.stats['errors'] += 1
            return None
    
    def sweep_servo(self, servo_id: int, speed: int = 10) -> bool:
        """Sweep servo 0-180-0 degrees."""
        print(f"\nTesting Servo {servo_id}:")
        print("  0° → 180° → 0°")
        
        positions = list(range(0, 181, speed)) + list(range(180, -1, -speed))
        
        for pos in positions:
            command = {
                'type': 'servo_move',
                'positions': {str(servo_id): pos}
            }
            
            response = self.send_command(command)
            
            if response and response.get('status') == 'ok':
                sys.stdout.write(f"\r  Position: {pos:3d}° (Response: OK)   ")
                sys.stdout.flush()
                time.sleep(0.05)
            else:
                print(f"\n  ✗ Failed at {pos}°")
                return False
        
        print(f"\n✓ Servo {servo_id} sweep complete")
        self.stats['sweep_success'] += 1
        return True
    
    def test_all_servos(self, speed: int = 20) -> bool:
        """Test all 5 servos sequentially."""
        success = True
        
        for servo_id in range(5):
            if not self.sweep_servo(servo_id, speed):
                success = False
                break
            time.sleep(0.5)
        
        return success
    
    def test_simultaneous(self) -> bool:
        """Test all servos moving simultaneously."""
        print("\nTesting Simultaneous Movement:")
        
        positions = list(range(0, 181, 20)) + list(range(160, -1, -20))
        
        for pos in positions:
            command = {
                'type': 'servo_move',
                'positions': {
                    '0': pos,
                    '1': 180 - pos,
                    '2': pos,
                    '3': 180 - pos,
                    '4': pos,
                }
            }
            
            response = self.send_command(command)
            
            if response and response.get('status') == 'ok':
                sys.stdout.write(f"\r  Position: {pos:3d}° (5 servos)   ")
                sys.stdout.flush()
                time.sleep(0.1)
            else:
                print(f"\n✗ Simultaneous test failed")
                return False
        
        print(f"\n✓ Simultaneous test complete")
        return True
    
    def test_speed(self) -> bool:
        """Test maximum servo speed."""
        print("\nTesting Speed (0° to 180° as fast as possible):")
        
        start_time = time.time()
        
        command = {
            'type': 'servo_move',
            'positions': {'0': 180}
        }
        
        response = self.send_command(command)
        
        if response and response.get('status') == 'ok':
            elapsed = time.time() - start_time
            print(f"✓ Response time: {elapsed*1000:.1f}ms")
            return True
        else:
            print("✗ Speed test failed")
            return False
    
    def test_response_time(self, iterations: int = 10) -> bool:
        """Measure average response time."""
        print(f"\nMeasuring Response Time ({iterations} iterations):")
        
        times = []
        
        for i in range(iterations):
            start = time.time()
            
            command = {'type': 'status'}
            response = self.send_command(command)
            
            elapsed = time.time() - start
            
            if response:
                times.append(elapsed * 1000)
                sys.stdout.write(f"\r  Iteration {i+1}/{iterations}: {elapsed*1000:.1f}ms   ")
                sys.stdout.flush()
            else:
                print(f"\n✗ Failed at iteration {i+1}")
                return False
        
        avg_time = sum(times) / len(times)
        max_time = max(times)
        min_time = min(times)
        
        print(f"\n✓ Avg: {avg_time:.1f}ms, Min: {min_time:.1f}ms, Max: {max_time:.1f}ms")
        return True
    
    def print_stats(self):
        """Print test statistics."""
        print("\n" + "="*50)
        print("TEST STATISTICS")
        print("="*50)
        print(f"Commands Sent: {self.stats['commands_sent']}")
        print(f"Responses Received: {self.stats['responses_received']}")
        print(f"Errors: {self.stats['errors']}")
        print(f"Sweeps Successful: {self.stats['sweep_success']}")
        
        if self.stats['commands_sent'] > 0:
            success_rate = (self.stats['responses_received'] / self.stats['commands_sent']) * 100
            print(f"Success Rate: {success_rate:.1f}%")
        
        print("="*50)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='SENTIO Servo Sweep Test',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python servo_sweep_test.py --port /dev/ttyUSB0 --servo 0
  python servo_sweep_test.py --port /dev/ttyUSB0 --all
  python servo_sweep_test.py --port /dev/ttyUSB0 --comprehensive
        '''
    )
    
    parser.add_argument('--port', required=True, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--servo', type=int, help='Test single servo (0-4)')
    parser.add_argument('--all', action='store_true', help='Test all servos sequentially')
    parser.add_argument('--simultaneous', action='store_true', help='Test simultaneous movement')
    parser.add_argument('--comprehensive', action='store_true', help='Run all tests')
    parser.add_argument('--speed', type=int, default=10, help='Sweep speed (degrees per step)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Serial baud rate')
    
    args = parser.parse_args()
    
    # Create tester
    tester = ServoTester(args.port, args.baudrate)
    
    # Connect
    if not tester.connect():
        sys.exit(1)
    
    try:
        # Run tests
        if args.comprehensive:
            tester.test_all_servos(args.speed)
            time.sleep(1)
            tester.test_simultaneous()
            time.sleep(1)
            tester.test_speed()
            time.sleep(1)
            tester.test_response_time()
        
        elif args.servo is not None:
            if not (0 <= args.servo <= 4):
                print(f"✗ Invalid servo ID: {args.servo} (must be 0-4)")
                sys.exit(1)
            tester.sweep_servo(args.servo, args.speed)
        
        elif args.all:
            tester.test_all_servos(args.speed)
        
        elif args.simultaneous:
            tester.test_simultaneous()
        
        else:
            parser.print_help()
            sys.exit(0)
    
    finally:
        tester.print_stats()
        tester.disconnect()
        print("✓ Test complete")


if __name__ == '__main__':
    main()
