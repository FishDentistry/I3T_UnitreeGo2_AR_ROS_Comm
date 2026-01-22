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
from ar_hrc_server_comm.encoder_senders.encoder_registry import get_encoder



        



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


        self.declare_parameter('transport_type', 'http')
        transport_type = self.get_parameter('transport_type').value
        

        self.img_enc_send = get_encoder("image",transport_type)

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1  # seconds of allowed timestamp difference
        )
        self.ts.registerCallback(self.sendRGBDepthImagesandMatricesToServer)
        # Get D435i intrinsics
        self.fx,self.fy,self.cx,self.cy = None,None,None,None
        self.intrinsics = {}
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
        self.intrinsics = {"fx":msg.k[0],"fy":msg.k[4],"cx":msg.k[2],"cy":msg.k[5]}
        
    
    def sendRGBDepthImagesandMatricesToServer(self,rgb,depth):
        # colorImage = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
        # depthImage = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
        # imagesURL = self.serverURL + "/segmentrobotview"
        # rgb_io = io.BytesIO()
        # PIL.Image.fromarray(colorImage).save(rgb_io, format='JPEG')
        # rgb_io.seek(0)

        # depth_io = io.BytesIO()
        # np.save(depth_io, depthImage)
        # depth_io.seek(0)

        
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

            # # Serialize transformation matrix and intrinsics as JSON
            #     matrixData = {
            #         "transformation_matrix": transMatrix.tolist(),
            #         "camera_intrinsics": {
            #             "fx": self.fx,
            #             "fy": self.fy,
            #             "cx": self.cx,
            #             "cy": self.cy
            #         }
            #     }
            #     meta_io = io.BytesIO()
            #     meta_io.write(json.dumps(matrixData).encode('utf-8'))
            #     meta_io.seek(0)

            #     files = {
            #         'colorImage': ('rgb.jpg', rgb_io, 'image/jpeg'),
            #         'depthImage': ('depth.npy', depth_io, 'application/octet-stream'),
            #         'matrixData': ('metadata.json', meta_io, 'application/json')
            #     }

            #     velData = {"linearVelMag":self.linearVelMag, "angularVelMag":self.angularVelMag}

                response = self.img_enc_send.encode_send(rgb, 'rgb8', self.get_namespace(), self.serverURL, depth_msg = depth,intrinsics=self.intrinsics,target_tf=transMatrix,vel_arr=[self.linearVelMag,self.angularVelMag])#requests.post(imagesURL, files=files, data = velData)
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