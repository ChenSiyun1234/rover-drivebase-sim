"""Assumed output-shaft fit to public motor operating points."""
from dataclasses import dataclass
import math
from .parameters import value as p

KT = p('rated_torque') / p('rated_current')
KE = p('rated_voltage') / (p('no_load_speed') * 2*math.pi/60)
RESISTANCE = (p('rated_voltage') - KE*p('rated_speed')*2*math.pi/60) / p('rated_current')
PEAK_TORQUE, PEAK_CURRENT = p('peak_torque'), p('peak_current')
FREE_INERTIA, FREE_DAMPING = p('free_inertia'), p('drive_damping')


def torque_limit(omega, voltage):
    """Symmetric motoring envelope; braking uses the same approximation."""
    return min(PEAK_TORQUE, KT*PEAK_CURRENT,
               KT*max(0.0, voltage-KE*abs(omega))/RESISTANCE)


def electrical(torque, omega, voltage):
    current = torque/KT
    power = max(0.0, current*current*RESISTANCE + torque*omega)
    return current, power, power/voltage


@dataclass
class Motor:
    kp: float = p('motor_kp')
    ki: float = p('motor_ki')
    integral: float = 0.0
    target: float = 0.0

    def demand(self, omega):
        return self.kp*(self.target-omega) + self.integral

    def accept(self, omega, torque, dt=p('electrical_dt')):
        error = self.target-omega
        requested = self.demand(omega)
        if abs(torque-requested) < 1e-10 or error*(requested-torque) <= 0:
            self.integral += self.ki*error*dt


@dataclass
class ProtectedMotor(Motor):
    behaviour: str = 'limit'
    current_setting: float = p('driver_current')
    timeout: float | None = None
    last_command: float = 0.0
    error: int = 1

    def enable(self, now):
        self.error, self.integral, self.target = 1, 0.0, 0.0
        self.last_command = now

    def command(self, target, now):
        self.target, self.last_command = target, now

    def check(self, omega, voltage, now):
        if self.error != 1:
            return 0.0
        if voltage < p('driver_undervoltage'):
            self.error = 9
        elif self.timeout is not None and now-self.last_command > self.timeout+1e-12:
            self.error = 13
        elif self.behaviour == 'trip' and abs(self.demand(omega))/KT > self.current_setting:
            self.error = 10
        return self.demand(omega) if self.error == 1 else 0.0

    def accept(self, omega, torque, dt=p('electrical_dt')):
        if self.error == 1:
            super().accept(omega, torque, dt)
