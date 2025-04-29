from typing import Tuple, Optional
from tactical_node.comm_msg import ComMsg

class CommInterfaceBase:
    def __init__(self):
        pass    

    def get_latest_message(self) -> Optional[ComMsg]:
        pass

    def start(self):
        pass

    def stop(self):
        pass