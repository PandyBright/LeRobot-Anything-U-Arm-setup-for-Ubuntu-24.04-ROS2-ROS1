#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import re
import numpy as np
from std_msgs.msg import Float64MultiArray

class ServoReaderNode(Node):
    def __init__(self):
        super().__init__('servo_reader_node')
        self.declare_parameter('baudrate', 115200)
        self.piper_pub = self.create_publisher(Float64MultiArray, '/piper_target_pose', 10)
        self.rate = self.create_rate(50)
        self.SERIAL_PORT = '/dev/ttyUSB0'

        self.BAUDRATE = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.ser = None
        self.zero_angles = [0.0] * 7
        self.initialized = False
        
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
            self.get_logger().info("Serial port opened")
            self._init_servos()
            self.initialized = True
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.get_logger().error(f"Try: sudo usermod -a -G dialout $USER")

    def send_command(self, cmd, expect_response=True, timeout=0.1):
        try:
            if not self.ser or not self.ser.is_open:
                return ""
            self.get_logger().debug(f"Sending: {cmd}")
            self.ser.write(cmd.encode('ascii'))
            time.sleep(0.008)
            
            if not expect_response:
                self.get_logger().debug(f"No response expected for: {cmd}")
                return ""
            
            start_time = time.time()
            response = ""
            while (time.time() - start_time) < timeout:
                data = self.ser.read(1)
                if not data:
                    time.sleep(0.001)
                    continue
                response += data.decode('ascii', errors='ignore')
            self.get_logger().debug(f"Received: {repr(response)}")
            return response
        except Exception as e:
            self.get_logger().warn(f"Serial communication error: {e}")
            return ""

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        self.get_logger().debug(f"PWM: {pwm_val}, Angle: {angle:.2f} (range: {pwm_min}-{pwm_max}, span: {angle_range}°)")
        return angle

    def _init_servos(self):
        self.send_command('#000PVER!', expect_response=True)
        for i in range(7):
            self.send_command("#000PCSK!", expect_response=False)
            self.send_command(f'#{i:03d}PULK!', expect_response=False)
            response = self.send_command(f'#{i:03d}PRAD!', expect_response=True)
            angle = self.pwm_to_angle(response.strip())
            self.zero_angles[i] = angle if angle is not None else 0.0
            self.get_logger().info(f"Servo {i} zero angle: {self.zero_angles[i]:.2f}° (PWM response: {response.strip()})")
        self.get_logger().info("Servo initial angle calibration completed")

    def run(self):
        if not self.initialized:
            self.get_logger().error("Node not initialized. Serial port access failed.")
            return
        
        self.get_logger().info("Starting main loop...")
        PUBLISH_RATE = 0.02  # 50 Hz = 20ms period

        while rclpy.ok():
            try:
                current_angles = [0.0] * 7
                
                # Read current angles from servos
                for i in range(7):
                    self.get_logger().debug(f"Reading servo {i}")
                    response = self.send_command(f'#{i:03d}PRAD!', expect_response=True)
                    self.get_logger().debug(f"Servo {i} response: {repr(response)}")
                    angle_deg = self.pwm_to_angle(response.strip())
                    if angle_deg is not None:
                        current_angles[i] = angle_deg
                        self.get_logger().debug(f"Servo {i}: {angle_deg:.2f}°")
                    else:
                        if response.strip():
                            self.get_logger().warn(f"Servo {i} response error: {response.strip()}")

                # Publish raw servo angles
                piper_msg = Float64MultiArray()
                piper_msg.data = current_angles
                self.piper_pub.publish(piper_msg)
                self.get_logger().info(f"Published servo angles: {[f'{x:.2f}' for x in current_angles]}")
                
                # Control publish rate using time.sleep
                self.get_logger().debug(f"Sleeping for {PUBLISH_RATE}s...")
                time.sleep(PUBLISH_RATE)
                self.get_logger().debug(f"Sleep done")
            except KeyboardInterrupt:
                self.get_logger().info("Interrupted by user")
                break
            except Exception as e:
                self.get_logger().error(f"Error in loop: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = ServoReaderNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
