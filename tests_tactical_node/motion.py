
class Motion:
    def __init__(self, ref_vel, max_acc):
        self.v = 0
        self.d = 0
        self.ref_vel = ref_vel
        self.acc = max_acc

    def get_position(self, time):
        if self.v < self.ref_vel:
             v = self.v + self.acc * time
             self.v = max(v, self.ref_vel)

        dist = self.v * time + 0.5 * self.acc * time**2
        return dist