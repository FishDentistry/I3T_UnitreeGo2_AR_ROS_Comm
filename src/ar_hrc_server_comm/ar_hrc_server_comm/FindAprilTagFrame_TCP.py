#!/usr/bin/env python3
import os
import socket
import json
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import apriltag  # <-- NEW

from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

from ament_index_python import get_package_share_directory
import yaml

# ---------------- exact function you provided ----------------
def transform_to_matrix(t):
    T = np.eye(4, dtype=np.float64)

    q = t.transform.rotation
    w, x, y, z = q.w, q.x, q.y, q.z  # fix the order!
    R = np.array([
            [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
    R = Rotation.from_quat(np.array([q.x, q.y, q.z, q.w ])).as_dcm()

    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z]
    return T
# --------------------------------------------------------------


class FindArucoFrameTCP(Node):
    def __init__(self):
        super().__init__('findArucoFrame_TCP')

        # --- Config from YAML (optional) ---
        # Now interpreted as TCP config instead of HTTP URL
        try:
            config_package = 'intel_realsense_functions'
            package_share_dir = get_package_share_directory(config_package)
            yaml_file = os.path.join(package_share_dir, "config", "config.yaml")
            with open(yaml_file, 'r') as f:
                config_data = yaml.safe_load(f)

            # New expected keys (with safe defaults):
            #   tcp_host: "127.0.0.1"
            #   tcp_port: 5001
            self.tcp_host = "192.168.1.227"  # config_data.get('tcp_host', '127.0.0.1')
            self.tcp_port = 5001             # int(config_data.get('tcp_port', 5001))
        except Exception:
            # Fallback defaults if config is missing or malformed
            self.tcp_host = '127.0.0.1'
            self.tcp_port = 5001

        self.get_logger().info(
            f"Configured TCP backend for tag frames at {self.tcp_host}:{self.tcp_port}"
        )

        # TCP socket (lazy connection)
        self._tcp_socket = None

        # --- Parameters ---
        self.declare_parameter('rgb_topic', '/realsense_rgb_image')
        self.declare_parameter('depth_topic', '/realsense_depth_image')
        # default as requested (these are RGB intrinsics published on this topic)
        self.declare_parameter('camera_info_topic', '/depth_camera_intrinsics')
        self.declare_parameter('camera_frame', 'camera_link')   # TF available for camera_link only
        self.declare_parameter('target_frame', 'map')

        # Kept name "aruco_min_side_px" for config compatibility, but now used for AprilTag
        self.declare_parameter('aruco_min_side_px', 20)

        # Tag size: 100 x 100 mm = 0.10 m
        self.declare_parameter('marker_length_m', 0.10)

        self.declare_parameter('depth_scale_m_per_unit', 0.001)  # 16UC1 (mm) -> meters
        self.declare_parameter('sample_window', 5)                # median window
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('debug_image_dir', '~/.ros/aruco_debug')

        # Resolve params
        self.rgb_topic   = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.marker_len  = self.get_parameter('marker_length_m').get_parameter_value().double_value

        self.depth_scale = self.get_parameter('depth_scale_m_per_unit').get_parameter_value().double_value
        self.sample_window = int(self.get_parameter('sample_window').get_parameter_value().integer_value)

        self.min_side_px = int(
            self.get_parameter('aruco_min_side_px').get_parameter_value().integer_value
        )

        raw_dir = self.get_parameter('debug_image_dir').get_parameter_value().string_value
        self.debug_image_dir = os.path.abspath(os.path.expanduser(raw_dir))
        self.save_debug_images = bool(self.get_parameter('save_debug_images').get_parameter_value().bool_value)
        if self.save_debug_images:
            os.makedirs(self.debug_image_dir, exist_ok=True)
            self.get_logger().info(f"Debug images → {self.debug_image_dir}")

        # --- Intrinsics from CameraInfo (must match the image used for detection) ---
        self.fx = self.fy = self.cx = self.cy = None
        self.dist_coeffs = None
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- CV / AprilTag ---
        self.bridge = CvBridge()

        # AprilTag configuration: family tag25h9, id 14
        self.tag_family = 'tag25h9'
        self.target_tag_id = 14

        at_options = apriltag.DetectorOptions(families=self.tag_family)
        self.apriltag_detector = apriltag.Detector(at_options)

        # Optical -> camera_link rotation (x_fwd=z_opt, y_left=-x_opt, z_up=-y_opt)
        self.A_opt_to_link = np.array([[ 0,  0, 1],
                                       [-1,  0, 0],
                                       [ 0, -1, 0]], dtype=np.float64)

        # --- Subscriptions (synced rgb+depth; need depth for center translation) ---
        self.rgb_sub = MFSubscriber(self, Image, self.rgb_topic, qos_profile=10)
        self.depth_sub = MFSubscriber(self, Image, self.depth_topic, qos_profile=10)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self._on_pair)

        # --- Publisher ---
        self.mat_pub = self.create_publisher(Float64MultiArray, 'marker_transform_matrix', 10)

        self.get_logger().info(
            "Listening on:\n"
            f"  RGB:    {self.rgb_topic}\n"
            f"  Depth:  {self.depth_topic}\n"
            f"  K from: {self.camera_info_topic}\n"
            f"  camera_frame (TF): {self.camera_frame}\n"
            f"  marker_length_m: {self.marker_len:.4f}\n"
            f"  AprilTag family: {self.tag_family}, ID: {self.target_tag_id}\n"
            f"Publishing: marker_transform_matrix\n"
            f"Sending over TCP to {self.tcp_host}:{self.tcp_port}"
        )

    # --- CameraInfo callback ---
    def _on_camera_info(self, msg: CameraInfo):
        self.fx = msg.k[0]; self.fy = msg.k[4]
        self.cx = msg.k[2]; self.cy = msg.k[5]
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d is not None and len(msg.d) > 0 else np.zeros(5)
        if self._camera_info_sub:
            self.destroy_subscription(self._camera_info_sub)
            self._camera_info_sub = None
            self.get_logger().info(
                f"Intrinsics set from {self.camera_info_topic}: "
                f"fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}, D len={len(self.dist_coeffs)}"
            )

    # --- Main callback ---
    def _on_pair(self, rgb_msg: Image, depth_msg: Image):
        if self.fx is None:
            return  # wait for intrinsics

        # Convert images
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # ----- AprilTag detection -----
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        detections = self.apriltag_detector.detect(gray)

        if len(detections) == 0:
            return

        # Filter by family + ID 14
        valid = []
        for d in detections:
            # tag_family can be bytes or str depending on version
            fam = d.tag_family.decode('utf-8') if isinstance(d.tag_family, bytes) else str(d.tag_family)
            if d.tag_id == self.target_tag_id and fam == self.tag_family:
                valid.append(d)

        if not valid:
            return

        # Extract corners from valid detections
        # d.corners: (4,2) array: [top-left, top-right, bottom-right, bottom-left]
        corners_list = [d.corners for d in valid]

        # Pick largest tag (by pixel area), with minimum side length constraint
        # _pick_largest_marker expects list of "c", where c[0] is (4,2), so wrap
        wrapped = [[c.astype(np.float32)] for c in corners_list]
        corners = self._pick_largest_marker(wrapped, self.min_side_px)
        if corners is None:
            return

        # --- PnP rotation in optical frame ---
        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]], dtype=np.float64)
        D = self.dist_coeffs if self.dist_coeffs is not None else np.zeros(5)

        obj_pts = np.array([
            [-self.marker_len/2,  self.marker_len/2, 0],
            [ self.marker_len/2,  self.marker_len/2, 0],
            [ self.marker_len/2, -self.marker_len/2, 0],
            [-self.marker_len/2, -self.marker_len/2, 0],
        ], dtype=np.float64)
        img_pts = corners.astype(np.float64)
        ok, rvec, tvec_opt = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return

        R_opt, _ = cv2.Rodrigues(rvec)

        # Convert rotation to camera_link
        A = self.A_opt_to_link
        R_link = A @ R_opt

        # --- Depth-based translation for tag center in camera_link ---
        center_px = np.mean(corners, axis=0)  # (u,v)
        z_raw = self._median_depth_at(depth, center_px, self.sample_window)
        if z_raw is None:
            return
        z_m = float(z_raw) if depth.dtype == np.float32 else float(z_raw) * self.depth_scale

        # optical-frame pinhole
        x_opt = (center_px[0] - self.cx) * z_m / self.fx
        y_opt = (center_px[1] - self.cy) * z_m / self.fy
        z_opt = z_m
        # convert to camera_link axes
        t_link = np.array([ z_opt, -x_opt, -y_opt ], dtype=np.float64)

        # --- Pose in camera_link ---
        H_link_marker = np.eye(4, dtype=np.float64)
        H_link_marker[:3, :3] = R_link
        H_link_marker[:3, 3]  = t_link

        # --- Compose to map: map <- camera_link <- marker ---
        try:
            t = self.tf_buffer.lookup_transform(self.target_frame, self.camera_frame,
                                                rgb_msg.header.stamp,
                                                rclpy.duration.Duration(seconds=0.1))
            T_map_link = transform_to_matrix(t)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({self.camera_frame} -> {self.target_frame}): {e}")
            return

        H_map_marker = T_map_link @ H_link_marker

        # Publish + send over TCP
        msg = Float64MultiArray()
        msg.data = H_map_marker.reshape(-1).tolist()
        self.mat_pub.publish(msg)
        self._post_matrix_async(H_map_marker)

        # Optional debug image
        if self.save_debug_images:
            self._save_debug_image(rgb, corners, rgb_msg.header.stamp, K, D, rvec, tvec_opt)

    # ---------- helpers ----------
    def _pick_largest_marker(self, corners_list, min_side_px: int):
        best = None; best_area = 0.0
        for c in corners_list:
            pts = c[0].astype(np.float32)  # (4,2) TL,TR,BR,BL
            side_lengths = [np.linalg.norm(pts[(i+1) % 4] - pts[i]) for i in range(4)]
            if min(side_lengths) < min_side_px:
                continue
            area = float(cv2.contourArea(pts))
            if area > best_area and np.isfinite(area):
                best_area = area
                best = pts.astype(np.float64)
        return best

    def _median_depth_at(self, depth_img, px, window):
        u, v = int(round(px[0])), int(round(px[1]))
        w = max(1, window | 1); r = w // 2
        u0, v0 = max(0, u - r), max(0, v - r)
        u1, v1 = min(depth_img.shape[1], u + r + 1), min(depth_img.shape[0], v + r + 1)
        patch = depth_img[v0:v1, u0:u1].astype(np.float32).flatten()
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _save_debug_image(self, bgr_image, corners, stamp, K, D, rvec, tvec_opt):
        try:
            img = bgr_image.copy()
            pts = corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [pts], True, (0, 255, 255), 2)

            tl, tr, br, bl = corners
            center_px    = np.mean(corners, axis=0)
            top_mid_px   = 0.5 * (tl + tr)
            right_mid_px = 0.5 * (tr + br)
            def put_point(p, label, color):
                u, v = int(round(p[0])), int(round(p[1]))
                cv2.circle(img, (u, v), 5, color, -1, lineType=cv2.LINE_AA)
                cv2.putText(img, label, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                return (u, v)
            c_uv = put_point(center_px, 'C', (0, 0, 255))
            t_uv = put_point(top_mid_px, 'T', (0, 255, 0))
            r_uv = put_point(right_mid_px, 'R', (255, 0, 0))
            cv2.line(img, c_uv, t_uv, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.line(img, c_uv, r_uv, (255, 0, 0), 1, cv2.LINE_AA)

            if hasattr(cv2.aruco, 'drawFrameAxes'):
                # Still fine to use for visualizing the pose
                cv2.drawFrameAxes(img, K, D if D is not None else np.zeros(5), rvec, tvec_opt, self.marker_len * 0.5)

            sec = getattr(stamp, 'sec', 0); nsec = getattr(stamp, 'nanosec', 0)
            path = os.path.join(self.debug_image_dir, f"apriltag_{sec}_{nsec}.png")
            cv2.imwrite(path, img)
            self.get_logger().info(f"Saved debug image: {path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save debug image: {e}")

    # ---------- TCP client helpers ----------

    def _ensure_tcp_connection(self):
        """
        Lazily connect or reconnect to the TCP server if needed.
        """
        if self._tcp_socket is not None:
            return

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # keep timeouts short so we don't block the callback forever
            s.settimeout(1.0)
            s.connect((self.tcp_host, self.tcp_port))
            self._tcp_socket = s
            self.get_logger().info(f"Connected to TCP server at {self.tcp_host}:{self.tcp_port}")
        except Exception as e:
            self.get_logger().warn(f"Could not connect to TCP server {self.tcp_host}:{self.tcp_port}: {e}")
            self._tcp_socket = None

    def _send_matrix_over_tcp(self, payload: dict):
        """
        Serialize the payload as JSON and send as a newline-terminated message
        to the TCP server. This should match what your Python TCP backend expects.
        """
        if self.tcp_host is None or self.tcp_port is None:
            return

        self._ensure_tcp_connection()
        if self._tcp_socket is None:
            # Connection failed; skip this send
            self.get_logger().debug("TCP send skipped: no connection.")
            return

        try:
            data = json.dumps(payload).encode('utf-8') + b'\n'
            self._tcp_socket.sendall(data)
        except Exception as e:
            self.get_logger().warn(f"TCP send failed, closing socket: {e}")
            try:
                self._tcp_socket.close()
            except Exception:
                pass
            self._tcp_socket = None  # will reconnect on next send

    def _post_matrix_async(self, H):
        """
        Formerly HTTP POST, now sends to a Python TCP server.

        Payload format (per message, newline-terminated JSON):
            {"namespace": "<ros_namespace>", "frameMatrix": [16 floats...]}
        """
        try:
            mat = H.reshape(-1).tolist()
        except Exception:
            self.get_logger().warn("TCP send skipped: matrix couldn't be flattened.")
            return

        payload = {
            "namespace": self.get_namespace(),
            "frameMatrix": mat
        }

        # Synchronous send over TCP (short timeout)
        self._send_matrix_over_tcp(payload)


def main():
    rclpy.init()
    node = FindArucoFrameTCP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up TCP socket if open
        if getattr(node, "_tcp_socket", None) is not None:
            try:
                node._tcp_socket.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
