from typing import Tuple, Optional

class CommInterfaceBase:
    def __init__(self):
        pass

    class Message:
        def __init__(self, time_stamp, front, back, vel, length=None):
            """

            :param time_stamp:
            :param front:
            :param back:
            :param vel:
            """
            self.time_stamp: int = time_stamp
            self.front: Tuple[float, float] = front
            self.back: Tuple[float, float] = back
            self.velocity: float = vel
            self.length: Optional[float, None] = length

    def get_latest_message(self) -> Optional[Message]:
        pass

    def start(self):
        pass

    def stop(self):
        pass