
class MalformedHexapodInfoFrameError(Exception):
    def __init__(self) -> None:
        super().__init__("Message length and/or check code given in INFO frame data was invalid!")

class IncompatibleHexapodProtocolVersionError(Exception):
    def __init__(self, handled_protocol_version: str, got_protocol_version: str) -> None:
        super().__init__(f"Current protocol version {handled_protocol_version} is incompatible with protocol version declared by Hardware Controller: {got_protocol_version}")


class OldHexapodProtocolVersionWarning(Warning):
    def __init__(self, handled_protocol_version: str, got_protocol_version: str) -> None:
        super().__init__(f"Current protocol version {handled_protocol_version} may not be angle to utilize all features provided by protocol version declared by Hardware Controller: {got_protocol_version}")


class NewHexapodProtocolVersionWarning(Warning):
    def __init__(self, handled_protocol_version: str, got_protocol_version: str) -> None:
        super().__init__(f"Current protocol version {handled_protocol_version} is newer than protocol version declared by Hardware Controller: {got_protocol_version}!\nConsider updating the Hardware Controller to the latest version!")