import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory
from message_filters import Subscriber, ApproximateTimeSynchronizer
import yaml 
import os
import io
import requests
import json
import numpy as np
import cv2
from cv_bridge import CvBridge
import PIL.Image
from sensor_msgs.msg import CameraInfo

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from math import sqrt


class RealsenseTopicsSubscriber(Node):

    def __init__(self):
        super().__init__('realsense_topics_subscriber')
        self.get_logger().info('Started')
        self.rgb_sub = Subscriber(self, Image, '/realsense_rgb_image')
        self.depth_sub = Subscriber(self, Image, '/realsense_depth_image')
        self.bridge = CvBridge()

        config_package = 'ar_hrc_server_comm'  # <- this is the name of the original package holding the YAML
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots

        # Load YAML config
        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)
        self.serverURL = config_data.get('server_url')

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.5  # seconds of allowed timestamp difference
        )
        self.ts.registerCallback(self.sendRGBDepthImagesandMatricesToServer)
        # Get D435i intrinsics
        self.fx,self.fy,self.cx,self.cy = None,None,None,None
        self.camIntrSub = self.create_subscription(CameraInfo, '/depth_camera_intrinsics', self.storeCameraIntr, 10)
        
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.serverResponse = None
        #self.create_timer(0.5,self.getPositionInWorld)
        #self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.linearVelMag = 0
        self.angularVelMag = 0
        self.odometrySub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',  #Or your odom topic
            self.odomCallback,
            10)
    
    def odomCallback(self,msg:Odometry):
        self.linearVelMag = sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 + msg.twist.twist.linear.z**2 )
        self.angularVelMag = sqrt(msg.twist.twist.angular.x**2 + msg.twist.twist.angular.y**2 + msg.twist.twist.angular.z**2 )
    
    def storeCameraIntr(self,msg: CameraInfo):
        self.fx,self.fy,self.cx,self.cy = msg.k[0],msg.k[4],msg.k[2],msg.k[5]
        

    # def sendRGBDepthImagesToServer(self,rgb, depth):
    #     #self.get_logger().info('Received synchronized RGB and depth images')
    #     colorImage = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
    #     depthImage = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
    #     imagesURL = self.serverURL + "/segmentrobotview"
    #     rgb_io = io.BytesIO()
    #     PIL.Image.fromarray(colorImage).save(rgb_io, format='JPEG')
    #     rgb_io.seek(0)

    #     depth_io = io.BytesIO()
    #     np.save(depth_io, depthImage)
    #     depth_io.seek(0)

    #     # Construct POST request
    #     files = {
    #         'colorImage': ('rgb.png', rgb_io, 'image/png'),
    #         'depthImage': ('depth.npy', depth_io, 'application/octet-stream')
    #     }
    #     try:
    #         response = requests.post(imagesURL, files=files)
    #         self.serverResponse = response.json()
    #         self.get_logger().info(str(self.serverResponse))
    #     except Exception as e:
    #         self.get_logger().warn(f"Could not post images to server: {e}")
    
    def sendRGBDepthImagesandMatricesToServer(self,rgb,depth):
        colorImage = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
        depthImage = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
        imagesURL = self.serverURL + "/segmentrobotview"
        rgb_io = io.BytesIO()
        PIL.Image.fromarray(colorImage).save(rgb_io, format='JPEG')
        rgb_io.seek(0)

        depth_io = io.BytesIO()
        np.save(depth_io, depthImage)
        depth_io.seek(0)

        
        try:
            now = rclpy.time.Time()
            if self.tfBuffer.can_transform('map', 'camera_link', now):
                transStamped = self.tfBuffer.lookup_transform(
                        target_frame='map',
                        source_frame='camera_link',
                        time=now,
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                transMatrix = self.getTransformationMatrix(transStamped)

            # Serialize transformation matrix and intrinsics as JSON
                matrixData = {
                    "transformation_matrix": transMatrix.tolist(),
                    "camera_intrinsics": {
                        "fx": self.fx,
                        "fy": self.fy,
                        "cx": self.cx,
                        "cy": self.cy
                    }
                }
                meta_io = io.BytesIO()
                meta_io.write(json.dumps(matrixData).encode('utf-8'))
                meta_io.seek(0)

                files = {
                    'colorImage': ('rgb.jpg', rgb_io, 'image/jpeg'),
                    'depthImage': ('depth.npy', depth_io, 'application/octet-stream'),
                    'matrixData': ('metadata.json', meta_io, 'application/json')
                }

                velData = {"linearVelMag":self.linearVelMag, "angularVelMag":self.angularVelMag}

                response = requests.post(imagesURL, files=files, data = velData)
                self.serverResponse = response.json()
                self.get_logger().info(str(self.serverResponse))
        except Exception as e:
            self.get_logger().warn(f"Could not post images to server: {e}")
    
    def getTransformationMatrix(self,transStamped):
        translation = transStamped.transform.translation
        rotation = transStamped.transform.rotation
        t = np.array([translation.x, translation.y, translation.z])

        r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_dcm()

        T = np.eye(4)
        T[0:3, 0:3] = r
        T[0:3, 3] = t
        return T
    
    # def getPositionInCameraFrame(self):
    #     if(self.serverResponse is not None):
    #         depthVal = self.serverResponse["depthVal"]
    #         pixelX,pixelY = self.serverResponse["centerPoint"][0],self.serverResponse["centerPoint"][1]
    #         #x = (pixelX - self.cx) * depthVal / self.fx
    #         #y = (pixelY- self.cy) * depthVal / self.fy
    #         #return x,y,depthVal
    #         z = (pixelY- self.cy) * depthVal / self.fy
    #         y = (pixelX - self.cx) * depthVal / self.fx
    #         x= depthVal
    #         return x,y,z

    
    # def getPositionInWorld(self):
    #     if(self.serverResponse is not None):
    #         camX,camY,camZ = self.getPositionInCameraFrame()
    #         try:
    #             now = rclpy.time.Time()
    #             self.get_logger().info(str(self.tfBuffer.can_transform('map', 'camera_link', now)))
    #             if self.tfBuffer.can_transform('map', 'camera_link', now):
    #                 trans = self.tfBuffer.lookup_transform(
    #                     target_frame='map',
    #                     source_frame='camera_link',
    #                     time=now,
    #                     timeout=rclpy.duration.Duration(seconds=1.0)
    #                 )
    #                 camera_point = PointStamped()
    #                 camera_point.header.frame_id = 'camera_link'
    #                 camera_point.header.stamp = now.to_msg()
    #                 camera_point.point.x = camX
    #                 camera_point.point.y = camY
    #                 camera_point.point.z = camZ

    #                 # Transform to map frame
    #                 world_point = tf2_geometry_msgs.do_transform_point(camera_point, trans)
    #                 self.get_logger().info("WOrld x: "+str(world_point.point.x))
    #                 self.get_logger().info("WOrld y: "+str(world_point.point.y))
    #                 self.get_logger().info("WOrld z: "+str(world_point.point.z))
    #                 marker = Marker()
    #                 marker.header.frame_id = 'map'
    #                 marker.header.stamp = self.get_clock().now().to_msg()
    #                 marker.ns = "world_point"
    #                 marker.id = 0
    #                 marker.type = Marker.SPHERE
    #                 marker.action = Marker.ADD

    #                 marker.pose.position.x = world_point.point.x
    #                 marker.pose.position.y = world_point.point.y
    #                 marker.pose.position.z = world_point.point.z
    #                 marker.pose.orientation.x = 0.0
    #                 marker.pose.orientation.y = 0.0
    #                 marker.pose.orientation.z = 0.0
    #                 marker.pose.orientation.w = 1.0

    #                 marker.scale.x = 0.1
    #                 marker.scale.y = 0.1
    #                 marker.scale.z = 0.1

    #                 marker.color.a = 1.0
    #                 marker.color.r = 1.0
    #                 marker.color.g = 0.0
    #                 marker.color.b = 0.0

    #                 self.marker_pub.publish(marker)

    #         except Exception as e:
    #             self.get_logger().warn(f"Transform unavailable: {e}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RealsenseTopicsSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()