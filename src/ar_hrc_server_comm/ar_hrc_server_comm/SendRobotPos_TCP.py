#!/usr/bin/env python3
import os
import socket
import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory
import yaml

from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped


class RobotPosTCP(Node):

    def __init__(self):
        super().__init__('send_rob_pos_tcp')

        # ---- Load TCP config from YAML ----
        config_package = 'ar_hrc_server_comm'  # name of the package holding the YAML
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(
            package_share_dir, "config", "config.yaml"
        )  # change the yaml file for different robots

        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)

        self.tcp_host = "192.168.1.227"  # config_data.get('tcp_host', '127.0.0.1')
        self.tcp_port = 5001             # int(config_data.get('tcp_port', 5001))

        self.get_logger().info(
            f"RobotStateComm will send position over TCP to {self.tcp_host}:{self.tcp_port}"
        )

        # TCP socket (lazy connection / reconnect on failure)
        self._tcp_socket = None

        # TF buffer/listener for map -> base_link
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # Timer to regularly check pose (10 Hz)
        self.timer = self.create_timer(0.01, self.sendRobotPose)

        # --- Rotation change gating setup ---
        # Maximum allowed angular speed (rad/s). Above this, position will NOT be sent.
        # Example: 1.0 rad/s ≈ 57.3 deg/s
        self.max_angular_speed_rad_per_sec = 0.6

        # Track last orientation and time to compute angular speed
        self._last_quat = None   # geometry_msgs.msg.Quaternion
        self._last_time = None   # rclpy.time.Time

    # ---------- TCP helpers ----------

    def _ensure_tcp_connection(self):
        """Create or re-create a TCP connection to the server if needed."""
        if self._tcp_socket is not None:
            return

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((self.tcp_host, self.tcp_port))
            self._tcp_socket = s
            self.get_logger().info(
                f"Connected to robot state TCP server at {self.tcp_host}:{self.tcp_port}"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not connect to TCP server {self.tcp_host}:{self.tcp_port}: {e}"
            )
            self._tcp_socket = None

    def _send_position_over_tcp(self, position):
        """
        Send position as JSON over TCP:

            {"position": [x, y, z]}\n
        """
        # Ensure we have a socket
        self._ensure_tcp_connection()
        if self._tcp_socket is None:
            return

        payload = {"position": position}
        try:
            data = (json.dumps(payload) + "\n").encode('utf-8')
            self._tcp_socket.sendall(data)
        except Exception as e:
            self.get_logger().warn(f"TCP send failed, closing socket: {e}")
            try:
                self._tcp_socket.close()
            except Exception:
                pass
            self._tcp_socket = None  # will reconnect on next send

    # ---------- Quaternion / rotation helpers ----------

    @staticmethod
    def _quat_to_tuple(q_msg):
        """Convert geometry_msgs/Quaternion to (x, y, z, w) tuple."""
        return (q_msg.x, q_msg.y, q_msg.z, q_msg.w)

    @staticmethod
    def _quat_angle_between(q1_msg, q2_msg):
        """
        Compute the absolute smallest angle between two quaternions (in radians).

        Uses:
            angle = 2 * acos(|dot(q1, q2)|)
        """
        x1, y1, z1, w1 = RobotPosTCP._quat_to_tuple(q1_msg)
        x2, y2, z2, w2 = RobotPosTCP._quat_to_tuple(q2_msg)

        dot = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2
        # Clamp due to numerical issues
        dot = max(-1.0, min(1.0, dot))
        angle = 2.0 * math.acos(abs(dot))
        return angle

    # ---------- Main pose updater ----------

    def sendRobotPose(self):
        """
        Look up the transform from map -> base_link and send only the position
        (x, y, z) to the TCP server, BUT skip sending if the rotation of
        base_link is changing faster than a configured threshold.
        """
        try:
            tf_time = rclpy.time.Time()  # "latest" for TF
            if self.tfBuffer.can_transform('map', 'base_link', tf_time):
                trans = self.tfBuffer.lookup_transform(
                    target_frame='map',
                    source_frame='base_link',
                    time=tf_time,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                # Current time and orientation
                now_time = self.get_clock().now()
                current_quat = trans.transform.rotation

                # If we have a previous orientation, compute angular speed
                if self._last_quat is not None and self._last_time is not None:
                    dt = (now_time - self._last_time).nanoseconds * 1e-9
                    if dt > 0.0:
                        angle = self._quat_angle_between(self._last_quat, current_quat)
                        angular_speed = angle / dt  # rad/s

                        if angular_speed > self.max_angular_speed_rad_per_sec:
                            # Update history but SKIP sending
                            self.get_logger().debug(
                                f"Skipping pose send: angular speed {angular_speed:.2f} rad/s "
                                f"> threshold {self.max_angular_speed_rad_per_sec:.2f} rad/s"
                            )
                            self._last_quat = current_quat
                            self._last_time = now_time
                            return

                # Update history (either first time or accepted motion)
                self._last_quat = current_quat
                self._last_time = now_time

                # If rotation is within threshold, send position
                position = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ]

                self._send_position_over_tcp(position)
            else:
                self.get_logger().debug("Transform map->base_link not available yet.")

        except Exception as e:
            self.get_logger().warn(f"Transform unavailable: {e}")

    def destroy_node(self):
        # Close TCP socket on shutdown
        if self._tcp_socket is not None:
            try:
                self._tcp_socket.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    robotStateComm = RobotPosTCP()

    try:
        rclpy.spin(robotStateComm)
    except KeyboardInterrupt:
        pass
    finally:
        robotStateComm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
