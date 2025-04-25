
class Motion:
    def __init__(self, max_acc):
        """
        max_acc : float
            The maximum magnitude of acceleration or deceleration [m/s²].
        """
        self.v = 0.0   # current velocity [m/s]
        self.d_total = 0.0   # total displacement [m]
        self.d = 0.0
        self.acc = max_acc

    def get_displacement(self, ref_speed, delta_t):
        """
        Advance the motion by a time-step delta_t, adjusting velocity
        toward ref_speed by at most acc*delta_t, then accumulate displacement.

        Parameters
        ----------
        ref_speed : float
            The desired target speed [m/s] for this time step.
        delta_t : float
            The time interval [s] over which to advance.

        Returns
        -------
        float
            The new total displacement [m].
        """
        v_old = self.v

        # accelerate or decelerate toward ref_speed:
        if self.v < ref_speed:
            v_new = min(v_old + self.acc * delta_t, ref_speed)
        elif v_old > ref_speed:
            v_new = max(v_old - self.acc * delta_t, ref_speed)
        else:
            v_new = v_old

        # step displacement under constant accel/decel:
        self.d = 0.5 * (v_old + v_new) * delta_t

        # update state
        self.v = v_new
        self.d_total += self.d

        return self.d, self.d_total