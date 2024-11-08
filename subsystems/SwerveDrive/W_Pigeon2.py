from phoenix6.hardware import Pigeon2

from wpimath.geometry import Rotation2d

class W_Pigeon2(Pigeon2):
    def get_rotation_2d(self):
        return Rotation2d().fromDegrees(self.get_yaw().value)