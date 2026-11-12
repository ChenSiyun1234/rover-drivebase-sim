"""Virtual-work load balance on a plane, with measured tangential forces.
Unknown loads act along world vertical; geometry and mass come from the scene.
"""

import mujoco
import numpy as np
from .drive import WHEELS


def solve_loads(vertical_virtual_work, known_virtual_work):
    """Solve A N + b = 0 without clipping infeasible negative predictions."""
    return np.linalg.solve(vertical_virtual_work, -np.asarray(known_virtual_work))


def predict_loads(model, data, points, tangential_forces):
    """Use dqL=-dqR to cancel the differential reaction.
    Fixed wheel spin keeps hub torque internal to each balance.
    """
    basis = np.zeros((model.nv, 6))
    basis[2, 0], basis[3, 1], basis[4, 2] = 1, 1, 1
    for col, name in ((3, "bogie_l"), (4, "bogie_r"), (5, "rocker_l")):
        basis[model.joint(name).dofadr[0], col] = 1
    basis[model.joint("rocker_r").dofadr[0], 5] = -1
    a, b = np.zeros((6, 6)), np.zeros(6)
    jac, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for i, wheel in enumerate(WHEELS):
        mujoco.mj_jac(model, data, jac, jacr, points[i], model.body("wheel_"+wheel).id)
        virtual = jac @ basis
        a[:, i] = virtual[2]
        b += virtual.T @ tangential_forces[i]
    for body in range(1, model.nbody):
        mujoco.mj_jacBodyCom(model, data, jac, jacr, body)
        b += (jac @ basis).T @ (model.body_mass[body] * model.opt.gravity)
    loads = solve_loads(a, b)
    return loads, a @ loads + b


def geometry(model, data):
    names = ("rocker_l", "rocker_r", "bogie_l", "bogie_r")
    # Zero joint offsets keep body origins at the pivots if a hinge is removed.
    return dict(pivots_world_m={n: data.xpos[model.body(n).id].tolist() for n in names},
                axles_world_m={w: data.xpos[model.body("wheel_"+w).id].tolist() for w in WHEELS},
                total_mass_kg=float(model.body_mass.sum()))
