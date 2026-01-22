import requests
import json
import numpy as np
from ar_hrc_server_comm.encoder_senders.TCPEncSenSuper import GenericTCPEncoder

class PointCloudHttpEncoderSender:
    endpoint = "/sendlidarpointcloud"
    def encode_send(self, points, namespace, url):
        cl_points = points
        if(isinstance(cl_points,np.ndarray)):
            cl_points = cl_points.tolist()
        
        payload = {"source_device":namespace, "points": cl_points}
        return requests.post(url+self.endpoint, json=payload)

        
        


class PointCloudTCPEncoderSender(GenericTCPEncoder):
        
    def encode_send(self, points, namespace, url):
        ip = self.extract_ip(url)
        pass
        # """
        # Send position as JSON over TCP:

        #     {"position": [x, y, z]}\n
        # """
        # # Ensure we have a socket
        # self._ensure_tcp_connection(url,self.tcp_port)
        # if self.tcp_socket is None:
        #     return

        # payload = {"position": position}
        # try:
        #     data = (json.dumps(payload) + "\n").encode('utf-8')
        #     self.tcp_socket.sendall(data)
        # except Exception as e:
        #     self.get_logger().warn(f"TCP send failed, closing socket: {e}")
        #     try:
        #         self.tcp_socket.close()
        #     except Exception:
        #         pass
        #     self.tcp_socket = None  # will reconnect on next send
