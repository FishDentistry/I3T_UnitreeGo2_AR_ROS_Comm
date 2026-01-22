# registry.py
from ar_hrc_server_comm.encoder_senders.Position_encoders_senders import PositionHttpEncoderSender, PositionTCPEncoderSender
from ar_hrc_server_comm.encoder_senders.Trajectory_encoders_senders import TrajectoryHttpEncoderSender, TrajectoryTCPEncoderSender
from ar_hrc_server_comm.encoder_senders.Img_encoders_senders import ImgHttpEncoderSender, ImgTCPEncoderSender


_ENCODERS = {
    ('position', 'http'): PositionHttpEncoderSender,
    ('position', 'tcp'): PositionTCPEncoderSender,
    ('trajectory', 'http'): TrajectoryHttpEncoderSender,
    ('trajectory', 'tcp'): TrajectoryTCPEncoderSender,
    ('image', 'http'): ImgHttpEncoderSender,
    ('image', 'tcp'): ImgTCPEncoderSender
}

def get_encoder(message_type, transport):
    key = (message_type, transport)
    try:
        return _ENCODERS[key]()
    except KeyError:
        valid = sorted(_ENCODERS.keys())
        raise ValueError(
            f"Invalid encoder selection: message_type='{message_type}', "
            f"transport='{transport}'. "
            f"Valid options are: {valid}"
        )

