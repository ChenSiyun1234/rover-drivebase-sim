"""Check fresh artifacts against physics accounting and summary definitions."""
import hashlib
import json
from pathlib import Path
import numpy as np
from sim.parameters import value as p
from sim.motor import KT, KE, RESISTANCE, PEAK_TORQUE, PEAK_CURRENT

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT/'results/summary.json').read_text())
    for name, digest in manifest['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest, name
    checked = []
    for summary in manifest['scenarios']:
        name = summary['name']
        saved = json.loads((ROOT/f'results/{name}.json').read_text())
        assert saved == summary
        with np.load(ROOT/f'results/{name}.npz', allow_pickle=False) as trace:
            for key in trace.files:
                assert np.isfinite(trace[key]).all(), (name, key)
            e, q, g, times = trace['electrical'], trace['qpos'], trace['gyro_rad_s'], trace['time_s']
            assert e.shape == (round(p('duration')/p('electrical_dt')), 27)
            assert len(q) == round(p('duration')/p('physics_dt'))
            assert np.allclose(np.diff(e[:,0]), p('electrical_dt'))
            assert np.allclose(np.diff(times), p('physics_dt'))
            assert np.allclose(np.linalg.norm(q[:,3:7], axis=1), 1, atol=1e-9)
            torque, speeds, demands, status = e[:,3:9], e[:,9:15], e[:,15:21], e[:,21:27]
            limit = np.minimum(min(PEAK_TORQUE, KT*PEAK_CURRENT, KT*p('driver_current')),
                               KT*np.maximum(0, e[:,1,None]-KE*np.abs(speeds))/RESISTANCE)
            assert np.all(np.abs(torque) <= limit+1e-7), name
            assert np.allclose(torque, np.clip(demands, -limit, limit), atol=1e-7), name
            power = np.maximum(0, (torque/KT)**2*RESISTANCE+torque*speeds)
            assert np.allclose(e[:,2], power.sum(axis=1)/e[:,1], atol=1e-8)
            assert np.allclose(e[:,1], p('pack_voltage')-p('pack_resistance')*e[:,2], atol=1e-6)
            assert np.max(np.abs(torque[status != 1]), initial=0) == 0
            assert abs(e[:,1].min()-summary['minimum_voltage_V']) < 1e-10
            assert abs(e[:,2].max()-summary['maximum_bus_current_A']) < 1e-10
            assert abs(e[:,2].mean()-summary['mean_bus_current_A']) < 1e-10
            assert np.allclose(q[-1,:3]-trace['start_position_m'], summary['displacement_m'], atol=1e-12)
            assert abs(np.max(np.abs(torque))/KT-summary['peak_winding_current_A']) < 1e-10
            tilt = np.degrees(np.arccos(np.clip(1-2*(q[:,4]**2+q[:,5]**2), -1, 1))).max()
            assert abs(tilt-summary['peak_tilt_deg']) < 1e-10
            from experiments.run_all import Case, load_scene
            model = load_scene(Case(**summary['case']))
            bogie_indices = [model.joint('bogie_'+s).qposadr[0] for s in 'lr']
            assert abs(np.degrees(np.abs(q[:,bogie_indices])).max()-summary['peak_bogie_angle_deg']) < 1e-10
            tail = times > p('duration')-p('steady_window')
            assert abs(np.degrees(g[tail,2].mean())-summary['mean_yaw_deg_s']) < 1e-10
            assert summary['fault_codes'] == status[-1].astype(int).tolist()
            for i, first in enumerate(summary['first_fault_times_s']):
                fault_rows = np.flatnonzero(status[:,i] != 1)
                actual = float(e[fault_rows[0],0]) if len(fault_rows) else None
                assert first == actual
            assert summary['peak_tilt_deg'] < 30, name
            if name == 'straight':
                assert summary['displacement_m'][0] > .9*p('duration')*p('straight_speed')
                assert abs(trace['vertical_load_N'][tail].sum(axis=1).mean()-50*p('gravity')) < 2
            if name == 'stall':
                assert np.max(np.abs(speeds[-100:])) < .001
            if name == 'watchdog':
                assert summary['fault_codes'] == [13]*6
                assert all(p('command_cutoff') < t < p('command_cutoff')+p('watchdog_timeout')+.01 for t in summary['first_fault_times_s'])
        checked.append(name)
    output = dict(passed=True, scenarios=checked,
                  checks=['source hashes', 'finite arrays', 'sample counts and timing',
                          'unit quaternions', 'motor envelope', 'torque clipping', 'bus power and sag',
                          'fault torque removal', 'summary reconstruction', 'pose stability',
                          'static weight support', 'stall locks', 'watchdog latching'])
    (ROOT/'results/validation.json').write_text(json.dumps(output, indent=2), encoding='utf-8')
    print(f'Validated {len(checked)} scenarios: accounting, summaries and fixtures passed')


if __name__ == '__main__':
    main()
