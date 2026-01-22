from typing import Any, Dict, Optional
import json
import socket
from urllib.parse import urlparse

JsonDict = Dict[str, Any]


class TCPClient:
    """
    TCP request/response client that can query for specific data types.

    Assumptions:
      - Server speaks length-prefixed JSON messages:
          [4-byte big-endian length][UTF-8 JSON bytes]
      - All TCP traffic uses port 5001
      - `.get(url, params=...)` is called similarly to `requests.get(...)`
      - The URL's endpoint is used ONLY to determine the request "type"
        (host from the URL may override self.host; path is mapped to a request type)

    Protocol (client -> server):
      {
        "request": "get_latest",
        "type": "<request_type>",
        "params": {...}
      }

    Protocol (server -> client):
      any JSON object (commonly: {"type": "<...>", "payload": {...}} or directly {"destination": ...})
    """

    DEFAULT_PORT = 5001

    # Map HTTP-style endpoints to a TCP request "type"
    ENDPOINT_TO_TYPE: Dict[str, str] = {
        "getoldestrobotdestination": "robot_destination",
        "getrobotstatus": "robot_status",
        # add more as needed:
        # "getwhatever": "whatever_type",
    }

    def __init__(
        self,
        host: str,
        timeout_s: float = 2.0,
        reconnect: bool = True,
        port: int = DEFAULT_PORT,
    ):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.reconnect = reconnect
        self._sock: Optional[socket.socket] = None

    def _connect(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout_s)
        s.connect((self.host, self.port))
        self._sock = s

    def _ensure_connected(self) -> None:
        if self._sock is None:
            self._connect()

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("TCP connection closed")
            buf += chunk
        return buf

    def _send_json(self, msg: JsonDict) -> None:
        assert self._sock is not None
        payload = json.dumps(msg, separators=(",", ":")).encode("utf-8")
        header = len(payload).to_bytes(4, "big")
        self._sock.sendall(header + payload)

    def _recv_one_json(self) -> JsonDict:
        assert self._sock is not None
        resp_len = int.from_bytes(self._recv_exact(4), "big")
        resp_bytes = self._recv_exact(resp_len)
        return json.loads(resp_bytes.decode("utf-8"))

    def _endpoint_from_url(self, url: str) -> str:
        """
        Extracts the last path segment as the endpoint name.
        Example: 'http://x/y/getoldestrobotdestination' -> 'getoldestrobotdestination'
        """
        parsed = urlparse(url)
        path = (parsed.path or "").strip("/")
        if not path:
            return ""
        return path.split("/")[-1]

    def get(
        self,
        url: str,
        params: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        request_type: Optional[str] = None,
    ) -> JsonDict:
        """
        Query the server for specific data.

        - `url`: may be an HTTP-like URL; host (if present) overrides self.host
        - `params`: optional dict of parameters (e.g., {"robotNamespace": "/go2"})
        - `request_type`: optional explicit request type (overrides endpoint mapping)
        - returns: decoded JSON dict response from server

        Usage patterns:
            data = tcp.get(commandURL, params={"robotNamespace": ns})
            data = tcp.get("http://192.168.1.50/ignored", request_type="robot_status", params={...})
        """
        parsed = urlparse(url)

        # Optional: override host if URL includes one
        if parsed.hostname:
            self.host = parsed.hostname

        # Optional: per-call timeout override
        old_timeout = self.timeout_s
        if timeout is not None:
            self.timeout_s = float(timeout)
            if self._sock is not None:
                try:
                    self._sock.settimeout(self.timeout_s)
                except Exception:
                    self._close()

        # Determine request type
        if request_type is None:
            endpoint = self._endpoint_from_url(url)
            request_type = self.ENDPOINT_TO_TYPE.get(endpoint)
            if request_type is None:
                raise ValueError(
                    f"Unknown endpoint '{endpoint}' for TCP client. "
                    f"Add it to ENDPOINT_TO_TYPE or pass request_type explicitly."
                )

        req_msg: JsonDict = {
            "request": "get_latest",
            "type": request_type,
            "params": params or {},
        }

        for attempt in (1, 2) if self.reconnect else (1,):
            try:
                self._ensure_connected()
                assert self._sock is not None

                # Send request and read response
                self._send_json(req_msg)
                return self._recv_one_json()

            except Exception:
                self._close()
                if attempt == 1 and self.reconnect:
                    continue
                raise

            finally:
                # Restore timeout if overridden
                if timeout is not None:
                    self.timeout_s = old_timeout
                    if self._sock is not None:
                        try:
                            self._sock.settimeout(self.timeout_s)
                        except Exception:
                            self._close()
