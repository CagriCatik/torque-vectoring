"""No additional yaw moment; baseline driving remains external."""


class OffYaw:
    def demand(self, state, reference, steering_rad):
        return 0.0
