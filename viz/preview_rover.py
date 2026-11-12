"""Render a still and animation by replaying the fresh obstacle trace."""
from pathlib import Path
import sys
import time
import json
import hashlib
import imageio.v2 as imageio
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.run_all import Case, load_scene


def main():
    started = time.perf_counter()
    summary = json.loads((ROOT/'results/obstacle.json').read_text())
    model = load_scene(Case(**summary['case']))
    data = mujoco.MjData(model)
    with np.load(ROOT/'results/obstacle.npz', allow_pickle=False) as trace:
        poses = trace['qpos']
        times = trace['time_s']
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 135, -23, 2.6
    frames = []
    with mujoco.Renderer(model, height=400, width=640) as renderer:
        for index in range(0, len(poses), 25):
            data.qpos[:] = poses[index]
            mujoco.mj_forward(model, data)
            camera.lookat[:] = data.qpos[:3] + [0.0, 0.0, -0.05]
            renderer.update_scene(data, camera=camera)
            frames.append(renderer.render().copy())
        imageio.mimsave(ROOT/'media/rover_drive.gif', frames, duration=50, loop=0)
        imageio.imwrite(ROOT/'media/rover.png', frames[len(frames)//2])
    files = ['media/rover_drive.gif', 'media/rover.png']
    report = dict(frames=len(frames), frame_period_s=float(times[25]-times[0]),
                  wall_seconds=time.perf_counter()-started,
                  renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256((ROOT/'results/obstacle.npz').read_bytes()).hexdigest(),
                  files={n: (ROOT/n).stat().st_size for n in files})
    (ROOT/'results/render.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
