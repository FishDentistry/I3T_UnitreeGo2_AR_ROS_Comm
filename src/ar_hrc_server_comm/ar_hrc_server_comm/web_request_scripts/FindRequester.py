from ar_hrc_server_comm.web_request_scripts.TCPClient import TCPClient
import requests


class RequestsWrapper:
    def __init__(self, timeout_s: float = 2.0):
        self.timeout_s = timeout_s
        self.session = requests.Session()

    def get(self, url, params=None, timeout=None, request_type=None):
        return self.session.get(url, timeout=self.timeout_s)


_REQUESTERS = {
    ('http'): RequestsWrapper,
    ('tcp'): TCPClient
}

def get_request_client_with_protocol(transport):
    key = (transport)
    try:
        return _REQUESTERS[key]()
    except KeyError:
        valid = sorted(_REQUESTERS.keys())
        raise ValueError(
            f"Invalid selection for request protocol: message_type='{transport}', ")