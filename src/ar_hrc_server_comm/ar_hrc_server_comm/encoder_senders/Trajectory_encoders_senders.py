import requests
import json
from ar_hrc_server_comm.encoder_senders.TCPEncSenSuper import GenericTCPEncoder


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

        
        


class TrajectoryTCPEncoderSender(GenericTCPEncoder):
    
    def encode_send(self, msg, namespace, url):
        """
        Send position as JSON over TCP:

            {"position": [x, y, z]}\n
        """
        trajectory = extractTrajectory(msg)
        ip = self.extract_ip(url)
        self._ensure_tcp_connection(ip,self.tcp_port)
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
