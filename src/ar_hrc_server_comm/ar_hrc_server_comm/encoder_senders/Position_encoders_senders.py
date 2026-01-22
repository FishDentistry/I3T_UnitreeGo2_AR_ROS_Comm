import requests
import socket
import json

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

        
        


class PositionTCPEncoderSender:
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
        
    def encode_send(self, position, orientation, namespace, url):
        """
        Send position as JSON over TCP:

            {"position": [x, y, z]}\n
        """
        # Ensure we have a socket
        self._ensure_tcp_connection(url,self.tcp_port)
        if self.tcp_socket is None:
            return

        payload = {"position": position}
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
