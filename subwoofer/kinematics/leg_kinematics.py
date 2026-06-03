import numpy as np

class LegKinematics:
    def __init__(self):
        self.upper_length = 0.108
        self.lower_length = 0.138

        self.boundary_scale = 0.9

    def leg_ik_2d(self, x: float, z: float) -> tuple[float, float]:
        L1 = self.upper_length
        L2 = self.lower_length

        D = np.sqrt(x*x + z*z)

        max_reach = self.boundary_scale * (self.upper_length + self.lower_length)
        min_reach = abs(self.upper_length - self.lower_length)

        if D > max_reach:
            scale = max_reach / D

            x *= scale
            z *= scale
        elif D < min_reach:
            scale = min_reach / D
            x *= scale
            z *= scale

        cos_knee = (L1*L1 + L2*L2 - D*D) / (2 * L1 * L2)
        cos_knee = max(-1, min(1, cos_knee))

        lower = np.pi - np.arccos(cos_knee)
        alpha = np.arctan2(z, x)

        cos_beta = (L1*L1 + D*D - L2*L2) / (2*L1*D)
        cos_beta = max(-1, min(1, cos_beta))
        beta = np.arccos(cos_beta)

        upper = alpha - beta

        return (upper, lower)

    def leg_ik(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        hip = np.arctan2(y, -z)
        r = np.sqrt(y*y + z*z)

        x_plane = x
        z_plane = -r

        leg_2d = self.leg_ik_2d(
            x_plane,
            z_plane
        )

        if leg_2d is None:
            return None

        upper_plane, lower_plane = leg_2d

        return (hip, upper_plane, lower_plane)