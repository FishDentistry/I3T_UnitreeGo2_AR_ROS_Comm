from urllib.parse import urlparse
import socket

class GenericTCPEncoder:
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

    def extract_ip(self,url: str) -> str:
        parsed = urlparse(url)
        host = parsed.hostname
        if host is None:
            raise ValueError(f"Invalid URL: {url}")
        return host