# import rclpy
# from rclpy.node import Node
# import rclpy.time
# from sensor_msgs.msg import PointCloud2
# import numpy as np
# import open3d as o3d
# import struct
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# from scipy.spatial.transform import Rotation
# import requests
# from ament_index_python import get_package_share_directory
# import yaml
# import os 
# import math





# class VoxelMapper(Node):
#     def __init__(self):
#         super().__init__('voxel_mapper')
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.livoxSub = self.create_subscription(
#             PointCloud2,
#             '/livox/lidar',  # Or your Livox topic
#             self.lidar_callback,
#             10)
#         # self.onboardLidarSub = self.create_subscription(
#         #     PointCloud2,
#         #     '/onboard_lidar_point_cloud2',  #Or your pointcloud2 topic
#         #     self.lidar_callback,
#         #     10)
#         self.voxel_size = 0.05  # meters
#         self.voxel_map = set()
#         self.all_points = []
#         self.get_logger().info("Voxel mapper initialized and listening...")
#         self.successCount = 0
#         config_package = 'ar_hrc_server_comm'  # <- this is the name of the original package holding the YAML
#         package_share_dir = get_package_share_directory(config_package)
#         yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots

#         # Load YAML config
#         with open(yaml_file, 'r') as f:
#             config_data = yaml.safe_load(f)
#         self.serverURL = config_data.get('server_url')
#         self.voxMapURL = self.serverURL +"/sendvoxelmap"
#         #self.timer = self.create_timer(0.1, self.checkRobotPose)
#         self.visitedPositions = []
#         self.currentPosition = None
#         self.numReadingsCurrPos = 0
    
    

    
#     def checkRobotPoseChanged(self):
#         try:
#             now = rclpy.time.Time()
#             if self.tf_buffer.can_transform('map', 'base_link', now):
#                 trans = self.tf_buffer.lookup_transform(
#                     target_frame='map',
#                     source_frame='base_link',
#                     time=now,
#                     timeout=rclpy.duration.Duration(seconds=1.0)
#                 )
                
#                 self.currentPosition = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
#                 for pos in self.visitedPositions:
#                     if(math.dist(pos,self.currentPosition) < 1.0):
#                         return False
                
#                 self.visitedPositions.append([self.currentPosition[0],self.currentPosition[1],self.currentPosition[2]])
#                 return True
                

#         except Exception as e:
#             self.get_logger().warn(f"Transform unavailable: {e}")
#             return False
        
    
#     def transformPointCloud(self,points, transform):
#         """Apply a TransformStamped to an Nx3 point cloud"""
#         # Extract translation
#         t = transform.transform.translation
#         trans = np.array([t.x, t.y, t.z])

#         # Extract rotation as a quaternion
#         q = transform.transform.rotation
#         w, x, y, z = q.w, q.x, q.y, q.z  # fix the order!
#         R = np.array([
#             [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
#             [    2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
#             [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
#         ])
#         R = Rotation.from_quat(np.array([q.x, q.y, q.z, q.w ])).as_dcm()

#         # Transform points
#         return (R @ points.T).T + trans
            

#     def lidar_callback(self, msg: PointCloud2):

#         if(self.numReadingsCurrPos < 3): #6
#             self.numReadingsCurrPos = self.numReadingsCurrPos + 1
#         else:
#             locChangedEnough = self.checkRobotPoseChanged()
#             if(locChangedEnough):
#                 self.numReadingsCurrPos = 0
#             else:
#                 self.get_logger().warn("Robot has not moved sufficiently")
#                 return
#         try:
#             transform = self.tf_buffer.lookup_transform(
#             target_frame='map',  # or 'odom' if no map
#             source_frame=msg.header.frame_id,
#             time=rclpy.time.Time(),  # latest available
#             timeout=rclpy.duration.Duration(seconds=1.0)
#             )
#             points = self.pointcloud2_to_xyz_array(msg)
#             xy_distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2 )
#             mask = xy_distances < 10.0
#             points = points[mask]
#             points = self.transformPointCloud(points, transform)
#             tf_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
#             msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#             if abs(tf_time - msg_time) > 0.2:  # more than 200ms apart
#                 self.get_logger().warn("Transform too old — skipping frame")
#                 self.get_logger().info(str(abs(tf_time - msg_time)))
#                 return
#             else:
#                 self.get_logger().info("Success")
#                 self.get_logger().info(str(self.successCount))
#         except Exception as e:
#             self.get_logger().warn(f"Transform failed: {e}")
#             return
#         #points = self.pointcloud2_to_xyz_array(msg)
#         self.all_points.append(points)

#         #Voxelize
#         indices = np.floor(points / self.voxel_size).astype(int)
#         for idx in indices:
#            self.voxel_map.add(tuple(idx))
#         #Save and send the map file
#         self.save_voxel_map()
#         self.sendVoxelMap()
        

#     def pointcloud2_to_xyz_array(self, cloud_msg):
#         # Basic unpacking of sensor_msgs/PointCloud2 to (N, 3) numpy array
#         fmt = 'fff'  # only x, y, z
#         width = cloud_msg.width
#         height = cloud_msg.height
#         point_step = cloud_msg.point_step
#         row_step = cloud_msg.row_step
#         data = cloud_msg.data
#         unpacker = struct.Struct(fmt)

#         points = []
#         for i in range(0, len(data), point_step):
#             x, y, z = unpacker.unpack_from(data, i)
#             if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
#                 points.append([x, y, z])
#         return np.array(points, dtype=np.float32)

#     def save_voxel_map(self):
#         self.get_logger().info("Saving voxel map...")

#         # Convert voxel indices back to centers
#         voxel_centers = np.array([np.array(v) * self.voxel_size + self.voxel_size / 2
#                                  for v in self.voxel_map])
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(voxel_centers)
#         pcd.paint_uniform_color([0.2, 0.6, 1.0])  # light blue

#         o3d.io.write_point_cloud("voxel_map.ply", pcd)
#         np.savez("voxel_map.npz", voxel_centers=voxel_centers)

#         self.get_logger().info("Saved voxel map to 'voxel_map.ply' and 'voxel_map.npz'.")
    
#     def sendVoxelMap(self):
#         files = {'voxMap': open('voxel_map.ply','rb')}
#         values = {'robotNamespace': self.get_namespace()}
#         try:
#             r = requests.post(self.voxMapURL, files=files, data=values, timeout=2.0)
#         except Exception as e:
#             self.get_logger().info("Could not post voxel map to server")
        

# def main(args=None):
#     rclpy.init(args=args)
#     mapper = VoxelMapper()

#     try:
#         rclpy.spin(mapper)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         #mapper.save_voxel_map()
#         mapper.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
# from rclpy.node import Node
# import rclpy.time
# from sensor_msgs.msg import PointCloud2
# import numpy as np
# import open3d as o3d
# import struct
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# from scipy.spatial.transform import Rotation
# import requests
# from ament_index_python import get_package_share_directory
# import yaml
# import os 
# import math





# class VoxelMapper(Node):
#     def __init__(self):
#         super().__init__('voxel_mapper')
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.livoxSub = self.create_subscription(
#             PointCloud2,
#             '/livox/lidar',  # Or your Livox topic
#             self.lidar_callback,
#             10)
#         # self.onboardLidarSub = self.create_subscription(
#         #     PointCloud2,
#         #     '/onboard_lidar_point_cloud2',  #Or your pointcloud2 topic
#         #     self.lidar_callback,
#         #     10)
#         self.voxel_size = 0.05  # meters
#         self.voxel_map = set()
#         self.all_points = []
#         self.get_logger().info("Voxel mapper initialized and listening...")
#         self.successCount = 0
#         config_package = 'ar_hrc_server_comm'  # <- this is the name of the original package holding the YAML
#         package_share_dir = get_package_share_directory(config_package)
#         yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots

#         # Load YAML config
#         with open(yaml_file, 'r') as f:
#             config_data = yaml.safe_load(f)
#         self.serverURL = config_data.get('server_url')
#         self.voxMapURL = self.serverURL +"/sendvoxelmap"
#         #self.timer = self.create_timer(0.1, self.checkRobotPose)
#         self.visitedPositions = []
#         self.currentPosition = None
#         self.numReadingsCurrPos = 0
    
    

    
#     def checkRobotPoseChanged(self):
#         try:
#             now = rclpy.time.Time()
#             if self.tf_buffer.can_transform('map', 'base_link', now):
#                 trans = self.tf_buffer.lookup_transform(
#                     target_frame='map',
#                     source_frame='base_link',
#                     time=now,
#                     timeout=rclpy.duration.Duration(seconds=1.0)
#                 )
                
#                 self.currentPosition = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
#                 for pos in self.visitedPositions:
#                     if(math.dist(pos,self.currentPosition) < 1.0):
#                         return False
                
#                 self.visitedPositions.append([self.currentPosition[0],self.currentPosition[1],self.currentPosition[2]])
#                 return True
                

#         except Exception as e:
#             self.get_logger().warn(f"Transform unavailable: {e}")
#             return False
        
    
#     def transformPointCloud(self,points, transform):
#         """Apply a TransformStamped to an Nx3 point cloud"""
#         # Extract translation
#         t = transform.transform.translation
#         trans = np.array([t.x, t.y, t.z])

#         # Extract rotation as a quaternion
#         q = transform.transform.rotation
#         w, x, y, z = q.w, q.x, q.y, q.z  # fix the order!
#         R = np.array([
#             [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
#             [    2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
#             [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
#         ])
#         R = Rotation.from_quat(np.array([q.x, q.y, q.z, q.w ])).as_dcm()

#         # Transform points
#         return (R @ points.T).T + trans
            

#     def lidar_callback(self, msg: PointCloud2):

#         if(self.numReadingsCurrPos < 3): #6
#             self.numReadingsCurrPos = self.numReadingsCurrPos + 1
#         else:
#             locChangedEnough = self.checkRobotPoseChanged()
#             if(locChangedEnough):
#                 self.numReadingsCurrPos = 0
#             else:
#                 self.get_logger().warn("Robot has not moved sufficiently")
#                 return
#         try:
#             transform = self.tf_buffer.lookup_transform(
#             target_frame='map',  # or 'odom' if no map
#             source_frame=msg.header.frame_id,
#             time=rclpy.time.Time(),  # latest available
#             timeout=rclpy.duration.Duration(seconds=1.0)
#             )
#             points = self.pointcloud2_to_xyz_array(msg)
#             xy_distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2 )
#             mask = xy_distances < 10.0
#             points = points[mask]
#             points = self.transformPointCloud(points, transform)
#             tf_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
#             msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#             if abs(tf_time - msg_time) > 0.2:  # more than 200ms apart
#                 self.get_logger().warn("Transform too old — skipping frame")
#                 self.get_logger().info(str(abs(tf_time - msg_time)))
#                 return
#             else:
#                 self.get_logger().info("Success")
#                 self.get_logger().info(str(self.successCount))
#         except Exception as e:
#             self.get_logger().warn(f"Transform failed: {e}")
#             return
#         #points = self.pointcloud2_to_xyz_array(msg)
#         self.all_points.append(points)

#         #Voxelize
#         indices = np.floor(points / self.voxel_size).astype(int)
#         for idx in indices:
#            self.voxel_map.add(tuple(idx))
#         #Save and send the map file
#         self.save_voxel_map()
#         self.sendVoxelMap()
        

#     def pointcloud2_to_xyz_array(self, cloud_msg):
#         # Basic unpacking of sensor_msgs/PointCloud2 to (N, 3) numpy array
#         fmt = 'fff'  # only x, y, z
#         width = cloud_msg.width
#         height = cloud_msg.height
#         point_step = cloud_msg.point_step
#         row_step = cloud_msg.row_step
#         data = cloud_msg.data
#         unpacker = struct.Struct(fmt)

#         points = []
#         for i in range(0, len(data), point_step):
#             x, y, z = unpacker.unpack_from(data, i)
#             if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
#                 points.append([x, y, z])
#         return np.array(points, dtype=np.float32)

#     def save_voxel_map(self):
#         self.get_logger().info("Saving voxel map...")

#         # Convert voxel indices back to centers
#         voxel_centers = np.array([np.array(v) * self.voxel_size + self.voxel_size / 2
#                                  for v in self.voxel_map])
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(voxel_centers)
#         pcd.paint_uniform_color([0.2, 0.6, 1.0])  # light blue

#         o3d.io.write_point_cloud("voxel_map.ply", pcd)
#         np.savez("voxel_map.npz", voxel_centers=voxel_centers)

#         self.get_logger().info("Saved voxel map to 'voxel_map.ply' and 'voxel_map.npz'.")
    
#     def sendVoxelMap(self):
#         files = {'voxMap': open('voxel_map.ply','rb')}
#         values = {'robotNamespace': self.get_namespace()}
#         try:
#             r = requests.post(self.voxMapURL, files=files, data=values, timeout=2.0)
#         except Exception as e:
#             self.get_logger().info("Could not post voxel map to server")
        

# def main(args=None):
#     rclpy.init(args=args)
#     mapper = VoxelMapper()

#     try:
#         rclpy.spin(mapper)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         #mapper.save_voxel_map()
#         mapper.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()







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

class VoxelMapper(Node):
    def __init__(self):
        super().__init__('voxel_mapper')
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
        self.get_logger().info("Voxel mapper initialized and listening...")
        self.successCount = 0

        config_package = 'ar_hrc_server_comm'
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(package_share_dir, "config", "config.yaml")

        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)
        self.serverURL = config_data.get('server_url')
        self.voxMapURL = self.serverURL + "/sendvoxelmap"

        # Retry configuration (can be exposed as ROS params if desired)
        self.max_retries = 5
        self.base_backoff = 0.75  # seconds
        self.backoff_factor = 1.7
        self.post_timeout = 2.5  # seconds

        self.visitedPositions = []
        self.currentPosition = None
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
                for pos in self.visitedPositions:
                    if (math.dist(pos, self.currentPosition) < 0.5):
                        return False

                self.visitedPositions.append([self.currentPosition[0],
                                              self.currentPosition[1],
                                              self.currentPosition[2]])
                return True

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
        if (self.numReadingsCurrPos < 3):
            self.numReadingsCurrPos += 1
        else:
            locChangedEnough = self.checkRobotPoseChanged()
            if (locChangedEnough):
                self.numReadingsCurrPos = 0
            else:
                self.get_logger().warn("Robot has not moved sufficiently")
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
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return

        self.all_points.append(points)

        # Voxelize
        indices = np.floor(points / self.voxel_size).astype(int)
        for idx in indices:
            self.voxel_map.add(tuple(idx))

        # Save and send the map file
        self.save_voxel_map()
        self.sendVoxelMap()

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

    def save_voxel_map(self):
        self.get_logger().info("Saving voxel map...")

        voxel_centers = np.array([np.array(v) * self.voxel_size + self.voxel_size / 2
                                  for v in self.voxel_map])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(voxel_centers)
        pcd.paint_uniform_color([0.2, 0.6, 1.0])  # light blue

        o3d.io.write_point_cloud("voxel_map.ply", pcd)
        np.savez("voxel_map.npz", voxel_centers=voxel_centers)

        self.get_logger().info("Saved voxel map to 'voxel_map.ply' and 'voxel_map.npz'.")

    # ---------- NEW: robust retry logic ----------
    def _should_retry_status(self, status_code: int) -> bool:
        # Retry on server errors and rate limiting
        return status_code >= 500 or status_code == 429

    def sendVoxelMap(self):
        """Attempt to POST the voxel map with retries and exponential backoff."""
        values = {'robotNamespace': self.get_namespace()}

        delay = self.base_backoff
        for attempt in range(1, self.max_retries + 1):
            try:
                with open('voxel_map.ply', 'rb') as f:
                    files = {'voxMap': f}
                    self.get_logger().info(f"Uploading voxel map (attempt {attempt}/{self.max_retries})...")
                    r = requests.post(self.voxMapURL, files=files, data=values, timeout=self.post_timeout)

                if r.ok:
                    self.get_logger().info(f"Voxel map upload succeeded (HTTP {r.status_code}).")
                    return
                else:
                    self.get_logger().warn(f"Server responded with HTTP {r.status_code}: {r.text[:200]}")
                    if not self._should_retry_status(r.status_code):
                        self.get_logger().warn("Not retryable status; giving up.")
                        return
            except requests.exceptions.RequestException as e:
                self.get_logger().warn(f"Upload error on attempt {attempt}: {e}")

            if attempt < self.max_retries:
                self.get_logger().info(f"Retrying in {delay:.2f}s...")
                time.sleep(delay)
                delay *= self.backoff_factor
            else:
                self.get_logger().error("Voxel map upload failed after maximum retry attempts.")

def main(args=None):
    rclpy.init(args=args)
    mapper = VoxelMapper()

    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






