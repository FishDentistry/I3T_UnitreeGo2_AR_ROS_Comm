import requests
import socket
import json



def extractTrajectory(path_msg):
    trajectory = []
    for poseStamped in path_msg.poses:
        position = [poseStamped.pose.position.x, poseStamped.pose.position.y,poseStamped.pose.position.z]
        trajectory.append(position)
    return trajectory

class TrajectoryHttpEncoderSender:
    endpoint = "/sendrobotcurrenttrajectory"
    def encode_send(self, msg, namespace, url):
        trajectory = extractTrajectory(msg)
        payload = {"namespace":namespace,"trajectory": trajectory}
        return requests.post(url+self.endpoint, json=payload)

        
        


class TrajectoryTCPEncoderSender:
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
        """
        Send position as JSON over TCP:

            {"position": [x, y, z]}\n
        """
        trajectory = extractTrajectory(msg)
        self._ensure_tcp_connection(url,self.tcp_port)
        if self.tcp_socket is None:
            return

        payload = {"trajectory": trajectory}
        try:
            data = (json.dumps(payload) + "\n").encode('utf-8')
            self.tcp_socket.sendall(data)
        except Exception as e:
            self.get_logger().warn(f"TCP send failed, closing socket: {e}")
            try:
                self.tcp_socket.close()
            except Exception:
                pass
            self.tcp_socket = None  # will reconnect on next send
