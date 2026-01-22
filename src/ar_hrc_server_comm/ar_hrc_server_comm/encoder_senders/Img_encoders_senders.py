import requests
import socket
import json
from cv_bridge import CvBridge
import io 
import PIL.Image
import numpy as np


class ImgHttpEncoderSender:
    endpoint = "/segmentrobotview"
    bridge = CvBridge()
    
    #Messages need namespace support
    def encode_send(self, rgb_msg, rgb_enc_type, namespace, url, depth_msg = None,intrinsics=None,target_tf=None,vel_arr=None):
        rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding=rgb_enc_type)
        rgb_io = io.BytesIO()
        PIL.Image.fromarray(rgb_cv).save(rgb_io, format='JPEG')
        rgb_io.seek(0)
        files = {'colorImage': ('rgb.jpg', rgb_io, 'image/jpeg')}
        #Check if a corresponding depth frame was also passed
        if(depth_msg is not None):
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            depth_io = io.BytesIO()
            np.save(depth_io, depth_cv)
            depth_io.seek(0)
            files["depthImage"] = ('depth.npy', depth_io, 'application/octet-stream')
        if(target_tf is not None and intrinsics is not None):
            # Serialize transformation matrix and intrinsics as JSON
                matrixData = {
                    "transformation_matrix": target_tf.tolist(),
                    "camera_intrinsics": {
                        "fx": intrinsics["fx"],
                        "fy": intrinsics["fy"],
                        "cx": intrinsics["cx"],
                        "cy": intrinsics["cy"]
                    }
                }
                meta_io = io.BytesIO()
                meta_io.write(json.dumps(matrixData).encode('utf-8'))
                meta_io.seek(0)
                files["matrixData"] = ('metadata.json', meta_io, 'application/json')
        if(vel_arr is not None):
            velData = {"linearVelMag":vel_arr[0], "angularVelMag":vel_arr[1]}
            response = requests.post(url+self.endpoint, files=files, data = velData)
            return response
        response = requests.post(url+self.endpoint, files=files)
        return response
        


        
        


class ImgTCPEncoderSender:
    tcp_port = 5001
    tcp_socket = None
    def _ensure_tcp_connection(self,url,port):
        """Create or re-create a TCP connection to the server if needed."""
        if self.tcp_socket is not None:
            return

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((url, port))
            self.tcp_socket = s
        except Exception as e:
            self.tcp_socket = None
        
    def encode_send(self, msg, namespace, url):
        pass
