import requests
import json
from ar_hrc_server_comm.encoder_senders.TCPEncSenSuper import GenericTCPEncoder

class PositionHttpEncoderSender:
    endpoint = "/sendrobotcurrentpose"
    def encode_send(self, position, orientation, namespace, url):
        # Extract position from the message

        # Build the JSON payload
        json_payload = {
            "namespace": namespace,
            "position": position,
            "orientation": orientation
        }
        return requests.post(url+self.endpoint, json=json_payload)

        
        


class PositionTCPEncoderSender(GenericTCPEncoder):
        
    def encode_send(self, position, orientation, namespace, url):
        """
        Send position as JSON over TCP:

            {"position": [x, y, z]}\n
        """
        ip = self.extract_ip(url)
        # Ensure we have a socket
        self._ensure_tcp_connection(ip,self.tcp_port)
        if self.tcp_socket is None:
            return

        payload = {"update_type":"pos_up","position": position}
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
