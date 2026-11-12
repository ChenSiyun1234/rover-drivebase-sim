"""Shared nonregenerating battery bus with assumed pack resistance."""
from dataclasses import dataclass
import numpy as np
from .motor import KE, KT, RESISTANCE, PEAK_TORQUE, PEAK_CURRENT
from .parameters import value as p


@dataclass
class Battery:
    open_circuit: float = p('pack_voltage')
    resistance: float = p('pack_resistance')
    undervoltage_threshold: float = p('bus_warning')
    minimum_voltage: float = float('inf')
    undervoltage: bool = False

    def solve(self, demands, speeds, current_limit=PEAK_CURRENT):
        """Return bus voltage/current and jointly feasible shaft torques."""
        demands, speeds = np.asarray(demands), np.asarray(speeds)
        ceiling = min(PEAK_TORQUE, KT*PEAK_CURRENT, KT*current_limit)

        def at(voltage):
            limit = np.minimum(ceiling, KT*np.maximum(0, voltage-KE*np.abs(speeds))/RESISTANCE)
            torque = np.clip(demands, -limit, limit)
            power = np.maximum(0, (torque/KT)**2*RESISTANCE + torque*speeds)
            current = power.sum()/voltage
            return voltage+self.resistance*current-self.open_circuit, torque, current

        lo, hi = self.open_circuit/2, self.open_circuit
        if at(lo)[0] > 0:
            raise RuntimeError('Battery operating point collapsed below half open-circuit voltage')
        for _ in range(p('battery_iterations')):
            mid = (lo+hi)/2
            if at(mid)[0] > 0:
                hi = mid
            else:
                lo = mid
        voltage = (lo+hi)/2
        _, torque, current = at(voltage)
        self.minimum_voltage = min(self.minimum_voltage, voltage)
        self.undervoltage |= voltage < self.undervoltage_threshold
        return voltage, current, torque
