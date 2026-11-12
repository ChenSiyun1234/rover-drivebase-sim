"""Rebuild reproducible motor, transport, obstacle and turn trials."""
from dataclasses import dataclass, asdict
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from sim.parameters import PARAMETERS, value as p
from sim.motor import ProtectedMotor, KT
from sim.power import Battery
from sim.can_dm import CanBus, encode_velocity
from sim.drive import Drive, WHEELS, YawRatePI, TorqueGovernor

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT/'viz/scene/rover_scene.xml'
OUT = ROOT/'results'


@dataclass(frozen=True)
class Case:
    controller: str = 'open'
    mu: float = p('friction')[0]
    velocity: float = 0.0
    yaw_deg_s: float = 0.0
    obstacle: bool = False
    stall: bool = False
    behaviour: str = 'limit'
    watchdog: bool = False


def cases():
    result = {'straight': Case(velocity=p('straight_speed')),
              'obstacle': Case(velocity=p('straight_speed'), obstacle=True),
              'stall': Case(stall=True),
              'watchdog': Case(velocity=p('straight_speed'), watchdog=True),
              'turn_trip': Case(mu=p('friction_sweep')[-1], yaw_deg_s=p('yaw_command'), behaviour='trip')}
    for mu in p('friction_sweep'):
        for controller in ('open', 'pi', 'governor'):
            result[f'turn_{mu}_{controller}'] = Case(controller=controller, mu=mu, yaw_deg_s=p('yaw_command'))
    return result


def scene_xml(case):
    root = ET.parse(SCENE).getroot()
    if not case.obstacle:
        world = root.find('worldbody')
        world.remove(world.find("geom[@name='obstacle']"))
    for geom in root.findall('.//geom'):
        geom.set('friction', f"{case.mu} {p('friction')[1]} {p('friction')[2]}")
    if case.stall:
        for wheel in WHEELS:
            ET.SubElement(root.find('equality'), 'joint', joint1='drive_'+wheel,
                          polycoef='0 0 0 0 0', solref=f"{p('physics_dt')} 1")
    return ET.tostring(root, encoding='unicode')


def load_scene(case):
    return mujoco.MjModel.from_xml_string(scene_xml(case))


def run_case(name, case):
    started = time.perf_counter()
    model = load_scene(case)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    vi = [model.joint('drive_'+w).dofadr[0] for w in WHEELS]
    qi = [model.joint('drive_'+w).qposadr[0] for w in WHEELS]
    gyro = model.sensor('gyro').adr[0]
    drive = Drive.from_scene(model, data)
    pi, governor = YawRatePI(), TorqueGovernor()
    battery, bus = Battery(), CanBus()
    motors = [ProtectedMotor(timeout=p('watchdog_timeout') if case.watchdog else None) for _ in WHEELS]
    voltage = p('pack_voltage')
    torque = np.zeros(6)
    electrical, poses, pose_times, velocities, loads, gyros = [], [], [], [], [], []
    fault_times = [None]*6
    command = np.zeros(6)
    dt, edt = p('physics_dt'), p('electrical_dt')
    settle_steps = round(p('settle_time')/dt)
    total_steps = settle_steps+round(p('duration')/dt)
    tire_ids = {model.geom('tire_'+w).id: i for i, w in enumerate(WHEELS)}
    for step in range(total_steps):
        elapsed = (step-settle_steps)*dt
        if step == settle_steps:
            start = data.qpos[:3].copy()
            for motor in motors:
                motor.enable(step*dt)
                motor.behaviour = case.behaviour
        if step % round(p('command_period')/dt) == 0:
            if elapsed < 0:
                command = np.zeros(6)
            elif case.stall:
                command = np.full(6, p('stall_target'))
            elif case.controller == 'pi':
                command = pi.speeds(drive, case.velocity, np.radians(case.yaw_deg_s),
                                    data.sensordata[gyro+2], voltage)
            else:
                yaw = np.radians(case.yaw_deg_s)
                if case.controller == 'governor':
                    yaw = governor.update(yaw, bus.feedback)
                command = drive.speeds(case.velocity, yaw)
        torques = []
        for substep in range(round(dt/edt)):
            now = step*dt+substep*edt
            t = elapsed+substep*edt
            if round(now/edt) % round(p('command_period')/edt) == 0 and not (case.watchdog and t >= p('command_cutoff')):
                for mid, target in enumerate(command, 1):
                    bus.send(encode_velocity(mid, float(target)), now)
            states = {mid: (float((data.qpos[qi[mid-1]]+np.pi) % (2*np.pi)-np.pi),
                            float(data.qvel[vi[mid-1]]), float(torque[mid-1]), motors[mid-1].error)
                      for mid in range(1, 7)}
            bus.advance(now, states)
            speeds = data.qvel[vi].copy()
            for mid, motor in enumerate(motors, 1):
                delivered = bus.last_command[mid]
                if delivered is not None and delivered > motor.last_command:
                    motor.command(bus.targets[mid], delivered)
            demands = [m.check(w, voltage, now) for m, w in zip(motors, speeds)]
            voltage, current, torque = battery.solve(demands, speeds, p('driver_current'))
            # Recheck protection after the shared bus sags.
            if voltage < p('driver_undervoltage'):
                for motor, w in zip(motors, speeds):
                    motor.check(w, voltage, now)
                voltage, current, torque = battery.solve([0]*6, speeds, p('driver_current'))
            for i, (motor, w, tau) in enumerate(zip(motors, speeds, torque)):
                motor.accept(w, tau)
                if t >= 0 and motor.error != 1 and fault_times[i] is None:
                    fault_times[i] = t
            torques.append(torque.copy())
            if t >= 0:
                electrical.append([t, voltage, current, *torque, *speeds, *demands,
                                   *[m.error for m in motors]])
        data.ctrl[:] = np.mean(torques, axis=0)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        if elapsed >= 0:
            pose_times.append(elapsed+dt)
            poses.append(data.qpos.copy())
            velocities.append(data.qvel.copy())
            gyros.append(data.sensordata[gyro:gyro+3].copy())
            vertical = np.zeros(6)
            for cid, contact in enumerate(data.contact):
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, cid, force)
                world_force = contact.frame.reshape(3, 3).T @ force[:3]
                for sign, gid in ((-1, contact.geom[0]), (1, contact.geom[1])):
                    if gid in tire_ids:
                        vertical[tire_ids[gid]] += sign*world_force[2]
            loads.append(vertical)
    e, q, g = np.array(electrical), np.array(poses), np.array(gyros)
    times = np.array(pose_times)
    tail = times > p('duration')-p('steady_window')
    final = q[-1, :3]-start
    summary = dict(name=name, case=asdict(case), mass_kg=float(model.body_mass.sum()),
                   duration_s=p('duration'), displacement_m=final.tolist(),
                   mean_yaw_deg_s=float(np.degrees(g[tail, 2].mean())),
                   minimum_voltage_V=float(e[:, 1].min()), maximum_bus_current_A=float(e[:, 2].max()),
                   mean_bus_current_A=float(e[:, 2].mean()),
                   peak_winding_current_A=float(np.max(np.abs(e[:, 3:9]))/KT),
                   fault_codes=[m.error for m in motors], first_fault_times_s=fault_times,
                   fault_count=sum(m.error != 1 for m in motors),
                   peak_bogie_angle_deg=float(np.degrees(np.abs(q[:, [model.joint('bogie_'+s).qposadr[0] for s in 'lr']])).max()),
                   peak_tilt_deg=float(np.degrees(np.arccos(np.clip(1-2*(q[:, 4]**2+q[:, 5]**2), -1, 1))).max()),
                   can_sent=bus.sent, can_dropped=bus.dropped, can_feedback_received=bus.received,
                   electrical_samples=len(e), physics_samples=len(q),
                   fixture_sha256=hashlib.sha256(scene_xml(case).encode()).hexdigest(),
                   wall_seconds=time.perf_counter()-started)
    np.savez_compressed(OUT/f'{name}.npz', electrical=e, time_s=times, qpos=q, start_position_m=start,
                        qvel=np.array(velocities), gyro_rad_s=g, vertical_load_N=np.array(loads))
    (OUT/f'{name}.json').write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    print(f"{name}: yaw {summary['mean_yaw_deg_s']:.3f} deg/s, min bus {summary['minimum_voltage_V']:.3f} V, faults {summary['fault_count']}", flush=True)
    return summary


def write_report(summaries):
    rows = ['# Results', '', 'I regenerate these trials with `python -m experiments.run_all`.', '',
            'Yaw is the mean body gyro z over the final steady window. Current and',
            'voltage summarize the entire recorded trial, excluding settling.', '',
            '| Case | Yaw (deg/s) | Forward displacement (m) | Min bus (V) | Peak bus (A) | Faults |',
            '|---|---:|---:|---:|---:|---:|']
    for s in summaries:
        rows.append(f"| {s['name']} | {s['mean_yaw_deg_s']:.3f} | {s['displacement_m'][0]:.3f} | {s['minimum_voltage_V']:.3f} | {s['maximum_bus_current_A']:.3f} | {s['fault_count']} |")
    rows += ['', 'Each JSON summary has a matching NPZ trace. Electrical columns are time_s,',
             'bus_voltage_V, bus_current_A, six torques_Nm, six speeds_rad_s, six',
             'requested_torques_Nm and six status codes, in lf/lm/lr/rf/rm/rr order.',
             'Physics arrays contain the full generalized pose and velocity, body gyro',
             'and summed world vertical contact force on each wheel.', '',
             'I place the obstacle under the left wheel path and lock wheel joints',
             'for the stall trial. The watchdog trial stops commands during motion.',
             'Trips latch on raw PI current requests; continuous limiting clips torque.', '',
             'Stall current follows the empirical motor fit, with unequal SI torque',
             'and back-emf constants. Contact uses rigid cylinders and compliant',
             'friction. See [Parameters](PARAMETERS.md) for equations and settings.', '',
             '![Controller comparison](../results/turns.png)', '']
    (ROOT/'docs/RESULTS.md').write_text('\n'.join(rows), encoding='utf-8')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for controller in ('open', 'pi', 'governor'):
        selected = [s for s in summaries if s['name'].startswith('turn_') and s['name'] != 'turn_trip' and s['case']['controller'] == controller]
        axes[0].plot([s['case']['mu'] for s in selected], [s['mean_yaw_deg_s'] for s in selected], 'o-', label=controller)
        axes[1].plot([s['case']['mu'] for s in selected], [s['mean_bus_current_A'] for s in selected], 'o-', label=controller)
    axes[0].axhline(p('yaw_command'), color='gray', linestyle='--', label='command')
    axes[0].set_ylabel('Mean yaw rate (deg/s)')
    axes[1].set_ylabel('Mean bus current (A)')
    for ax in axes:
        ax.set_xlabel('Sliding friction coefficient'); ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(OUT/'turns.png', dpi=130); plt.close(fig)


def main():
    started = time.perf_counter()
    OUT.mkdir(exist_ok=True)
    summaries = [run_case(name, case) for name, case in cases().items()]
    write_report(summaries)
    sources = ['sim/parameters.py', 'sim/motor.py', 'sim/power.py', 'sim/can_dm.py',
               'sim/drive.py', 'viz/build_rover.py', 'experiments/run_all.py', 'viz/scene/rover_scene.xml']
    payload = dict(scenarios=summaries, parameters=PARAMETERS,
                   source_sha256={n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in sources},
                   numpy_version=np.__version__, mujoco_version=mujoco.__version__,
                   wall_seconds=time.perf_counter()-started)
    (OUT/'summary.json').write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
    print(f"Generated {len(summaries)} scenarios in {payload['wall_seconds']:.2f} s", flush=True)


if __name__ == '__main__':
    main()
