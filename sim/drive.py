"""Generic differential wheel targets and optional yaw controllers."""

from dataclasses import dataclass
import numpy as np
from .motor import KE
from .parameters import value as p

WHEELS = tuple(f"{side}{tag}" for side in "lr" for tag in "fmr")


@dataclass
class Drive:
    separation: float
    radius: float
    slip_multiplier: float = p('kinematic_multiplier')

    @classmethod
    def from_scene(cls, model, data):
        """Read wheel centres after forward kinematics; multiplier is assumed."""
        left = data.xpos[model.body("wheel_lf").id]
        right = data.xpos[model.body("wheel_rf").id]
        radius = float(model.geom_size[model.geom("tire_lf").id, 0])
        return cls(float(np.linalg.norm(left - right)), radius)

    def speeds(self, velocity: float, yaw_rate: float):
        half_turn = yaw_rate * self.separation * self.slip_multiplier / 2
        return np.repeat([(velocity - half_turn) / self.radius,
                          (velocity + half_turn) / self.radius], 3)


@dataclass
class YawRatePI:
    """Gyro yaw feedback; anti-windup follows wheel-speed clipping."""

    kp: float = p('yaw_kp')
    ki: float = p('yaw_ki')
    period: float = p('command_period')
    noise_std: float = p('gyro_noise')
    seed: int = p('seed')
    integral: float = 0.0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.measurement = 0.0

    def speeds(self, drive, velocity, yaw_rate, gyro_z, voltage):
        self.measurement = gyro_z + self.rng.normal(0.0, self.noise_std)
        error = yaw_rate - self.measurement
        requested = drive.speeds(velocity, yaw_rate + self.kp * error + self.integral)
        limit = max(0.0, voltage) / KE
        targets = np.clip(requested, -limit, limit)
        # Freeze integration if yaw error pushes farther into speed saturation.
        blocked = error * np.mean((requested - targets)[3:] -
                                  (requested - targets)[:3]) > 0
        if not blocked:
            self.integral += self.ki * error * self.period
        return targets


@dataclass
class TorqueGovernor:
    """Reduce yaw demand from CAN torque feedback; recover slowly below load."""

    limit_torque: float = p('driver_current')
    scale: float = 1.0
    period: float = p('command_period')

    def update(self, yaw, feedback):
        if len(feedback) != 6 or any(v["error"] != 1 for v in feedback.values()):
            self.scale = 0.0
            return 0.0
        ratio = max(abs(v["torque"]) for v in feedback.values()) / self.limit_torque
        if ratio > p('governor_high'):
            self.scale *= max(0.0, 1-p('governor_decay')*self.period*(ratio/p('governor_high')-1))
        elif ratio < p('governor_low'):
            self.scale = min(1.0, self.scale+p('governor_recovery')*self.period)
        return yaw * self.scale
