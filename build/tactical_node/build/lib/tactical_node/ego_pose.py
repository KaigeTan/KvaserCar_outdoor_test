import tactical_node.parameters as parameters



class EgoPose:
    def __init__(self,
                 front_x: float,
                 front_y:float,
                 rear_x:float,
                 rear_y:float,
                 vel_x:float,
                 vel_y:float) -> None:
        self.front_x:float = front_x
        self.front_y:float = front_y
        self.rear_x:float = rear_x
        self.rear_y:float = rear_y
        self.vel_x:float = vel_x
        self.vel_y:float = vel_y
        self.vel_fw:float = vel_x
        # TODO use proper forward vector to get forward acceleration
        self.acc_fw:float = parameters.EGO_MAX_ACC