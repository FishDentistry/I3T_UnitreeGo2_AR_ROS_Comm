#

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation
import requests
from ament_index_python import get_package_share_directory
import yaml
import os
import math
import time  # <-- added
from ar_hrc_server_comm.encoder_senders.encoder_registry import get_encoder

class CollSendLivoxCloud(Node):
    def __init__(self):
        super().__init__('coll_send_livox_cloud')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.livoxSub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.lidar_callback,
            10)

        self.voxel_size = 0.05  # meters
        self.voxel_map = set()
        self.all_points = []
        self.get_logger().info("Livox PC listener active")
        self.successCount = 0

        config_package = 'ar_hrc_server_comm'
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(package_share_dir, "config", "config.yaml")

        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)
        self.serverURL = config_data.get('server_url')

        self.declare_parameter('transport_type', 'http')
        transport_type = self.get_parameter('transport_type').value
        self.cloud_enc_send = get_encoder("point_cloud",transport_type)
        

        # Retry configuration (can be exposed as ROS params if desired)
        self.max_retries = 5
        self.base_backoff = 0.75  # seconds
        self.backoff_factor = 1.7
        self.post_timeout = 2.5  # seconds

        self.visitedPositions = []
        self.lastPosition = None
        self.currentPosition = None
        self.pos_diff_thresh = 0.5
        self.numReadingsCurrPos = 0

    def checkRobotPoseChanged(self):
        try:
            now = rclpy.time.Time()
            if self.tf_buffer.can_transform('map', 'base_link', now):
                trans = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame='base_link',
                    time=now,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                self.currentPosition = [trans.transform.translation.x,
                                        trans.transform.translation.y,
                                        trans.transform.translation.z]
                
                if(self.lastPosition is None):
                    return True
                if (math.dist(self.lastPosition, self.currentPosition) < self.pos_diff_thresh):
                    return False

        except Exception as e:
            self.get_logger().warn(f"Transform unavailable: {e}")
            return False

    def transformPointCloud(self, points, transform):
        """Apply a TransformStamped to an Nx3 point cloud"""
        t = transform.transform.translation
        trans = np.array([t.x, t.y, t.z])

        q = transform.transform.rotation
        R = Rotation.from_quat(np.array([q.x, q.y, q.z, q.w])).as_dcm()

        return (R @ points.T).T + trans

    def lidar_callback(self, msg: PointCloud2):
        locChangedEnough = self.checkRobotPoseChanged()
        if (locChangedEnough == False):
                return
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            points = self.pointcloud2_to_xyz_array(msg)
            xy_distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
            mask = xy_distances < 12.0
            points = points[mask]
            points = self.transformPointCloud(points, transform)
            tf_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            if abs(tf_time - msg_time) > 0.2:
                self.get_logger().warn("Transform too old — skipping frame")
                self.get_logger().info(str(abs(tf_time - msg_time)))
                return
            else:
                self.get_logger().info("Success")
                self.get_logger().info(str(self.successCount))
                self.lastPosition = self.currentPosition
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return

        #self.all_points.append(points)

        # Voxelize
        #indices = np.floor(points / self.voxel_size).astype(int)
        #for idx in indices:
            #self.voxel_map.add(tuple(idx))

        # Save and send the map file
        #self.save_voxel_map()
        self.sendPointCloud(points)

    def pointcloud2_to_xyz_array(self, cloud_msg):
        fmt = 'fff'  # x, y, z
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        unpacker = struct.Struct(fmt)

        points = []
        for i in range(0, len(data), point_step):
            x, y, z = unpacker.unpack_from(data, i)
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])
        return np.array(points, dtype=np.float32)


    def sendPointCloud(self,points):
        response = self.cloud_enc_send.encode_send(points,self.get_namespace(),self.serverURL)

def main(args=None):
    rclpy.init(args=args)
    mapper = CollSendLivoxCloud()

    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






