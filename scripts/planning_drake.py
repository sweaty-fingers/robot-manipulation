"""Drake examples for docs `docs/background/17_trajectories_exercises.md`: door-handle IK with minimal constraints on an
iiwa, and graph-of-convex-sets (GCS) trajectory optimization through convex regions.

    .venv/bin/python scripts/planning_drake.py ik     # handle grasp + door sweep (downloads the iiwa model once)
    .venv/bin/python scripts/planning_drake.py gcs    # GCS on the diamond-obstacle toy problem
"""

from __future__ import annotations

import sys

import numpy as np

IIWA = "package://drake_models/iiwa_description/urdf/iiwa14_no_collision.urdf"
Q0 = np.array([0.0, 0.6, 0.0, -1.4, 0.0, 1.0, 0.0])  # "comfortable" joint-centering pose


def make_plant():
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant
    from pydrake.multibody.tree import FixedOffsetFrame

    plant = MultibodyPlant(0.0)
    Parser(plant).AddModels(url=IIWA)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))
    # gripper frame G: x = closing direction, y = approach, z = along the finger pads (the line the handle must lie on)
    R7G = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)  # columns: G.x = y7, G.y = z7, G.z = x7
    plant.AddFrame(FixedOffsetFrame("G", plant.GetFrameByName("iiwa_link_7"), RigidTransform(RotationMatrix(R7G), [0, 0, 0.05])))
    plant.Finalize()
    return plant


def solve_ik(plant, handle, mode, q_init=Q0):
    """mode 'minimal': two points of the gripper centre line lie on the handle axis segment (orientation about it free).
    mode 'full': gripper pose fixed (approach along +x, closing along -y)."""
    from pydrake.math import RotationMatrix
    from pydrake.multibody.inverse_kinematics import InverseKinematics
    from pydrake.solvers import Solve

    ik = InverseKinematics(plant, with_joint_limits=True)
    G, W = plant.GetFrameByName("G"), plant.world_frame()
    if mode == "minimal":
        for dz in (-0.02, 0.02):
            # x, y exact (on the bar's axis); z may slide +-0.10 m along the bar
            lo, hi = handle + [0, 0, -0.10], handle + [0, 0, 0.10]
            lo[:2], hi[:2] = handle[:2], handle[:2]
            ik.AddPositionConstraint(frameB=G, p_BQ=[0, 0.10, dz], frameA=W, p_AQ_lower=lo, p_AQ_upper=hi)
    else:
        Rd = RotationMatrix(np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float))  # columns: G.x=-y_w, G.y=+x_w, G.z=z_w
        p = handle - Rd.matrix()[:, 1] * 0.10
        ik.AddPositionConstraint(frameB=G, p_BQ=[0, 0, 0], frameA=W, p_AQ_lower=p, p_AQ_upper=p)
        ik.AddOrientationConstraint(frameAbar=W, R_AbarA=Rd, frameBbar=G, R_BbarB=RotationMatrix(), theta_bound=1e-3)
    q = ik.q()
    ik.prog().AddQuadraticErrorCost(np.eye(7), Q0, q)
    ik.prog().SetInitialGuess(q, q_init)
    r = Solve(ik.prog())
    return (r.GetSolution(q) if r.is_success() else None), r


def pose_of_G(plant, q):
    ctx = plant.CreateDefaultContext()
    plant.SetPositions(ctx, q)
    return plant.CalcRelativeTransform(ctx, plant.world_frame(), plant.GetFrameByName("G"))


def run_ik():
    plant = make_plant()
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(0.35, 0.75, 60), rng.uniform(-0.45, 0.45, 60), rng.uniform(0.25, 0.75, 60)])
    print("== handle grasp on 60 random handle positions (vertical bar, 0.2 m)")
    res = {}
    for mode in ("minimal", "full"):
        ok = 0
        tilt = []
        for h in pts:
            q, r = solve_ik(plant, h, mode)
            if q is not None:
                ok += 1
                X = pose_of_G(plant, q)
                tilt.append(np.degrees(np.arccos(np.clip(X.rotation().matrix()[:, 1] @ np.array([1.0, 0, 0]), -1, 1))))
        res[mode] = (ok, tilt)
        print(f"  {mode:8s}: feasible {ok}/60" + (f", approach direction deviates from +x by {np.mean(tilt):.0f} deg on average (max {np.max(tilt):.0f})" if tilt else ""))
    print("== one handle, minimal constraints: what the solver does with the freedom")
    h = np.array([0.5, 0.1, 0.45])
    q, r = solve_ik(plant, h, "minimal")
    print("  feasible:", q is not None)
    if q is not None:
        X = pose_of_G(plant, q)
        for dz in (-0.02, 0.02):
            pw = X @ np.array([0, 0.10, dz])
            print(f"  point z{dz:+.2f}: world {np.round(pw, 4)}  (handle axis x,y = {h[:2]})")
        print("  joint distance from comfort pose:", round(float(np.linalg.norm(q - Q0)), 3))
    print("== door sweep: handle moves on an arc about a vertical hinge at (0.35, 0.45)")
    hinge = np.array([0.35, 0.45, 0.0])
    qprev = Q0
    for th in range(0, 71, 10):
        t = np.radians(th)
        hp = hinge + np.array([0.4 * np.sin(t), -0.4 * np.cos(t), 0.40])
        q, r = solve_ik(plant, hp, "minimal", qprev)
        if q is None:
            print(f"  door {th:2d} deg: infeasible")
            continue
        step = float(np.max(np.abs(q - qprev)))
        X = pose_of_G(plant, q)
        yaw = np.degrees(np.arctan2(X.rotation().matrix()[1, 1], X.rotation().matrix()[0, 1]))
        print(f"  door {th:2d} deg: handle {np.round(hp, 3)}  feasible, approach heading {yaw:6.1f} deg, max joint step {step:.3f} rad" + ("  (from the comfort pose)" if th == 0 else ""))
        qprev = q


def run_gcs():
    from pydrake.geometry.optimization import GraphOfConvexSetsOptions, HPolyhedron, Point
    from pydrake.planning import GcsTrajectoryOptimization

    B = 2.0
    box = lambda: (np.array([[1, 0], [-1, 0], [0, 1], [0, -1]], float), np.array([B] * 4, float))
    regs = []
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):  # half-plane  sx*x + sy*y >= 1  (outside the diamond)
        A, b = box()
        A = np.vstack([A, [-sx, -sy]])
        b = np.append(b, -1.0)
        regs.append(HPolyhedron(A, b))
    names = ["x+y>=1", "-x+y>=1", "x-y>=1", "-x-y>=1"]
    start, goal = np.array([-1.5, 0.4]), np.array([1.5, 0.4])
    gcs = GcsTrajectoryOptimization(2)
    g = gcs.AddRegions(regs, order=1)
    s = gcs.AddRegions([Point(start)], order=0)
    t = gcs.AddRegions([Point(goal)], order=0)
    gcs.AddEdges(s, g)
    gcs.AddEdges(g, t)
    gcs.AddEdges(g, g)  # regions that intersect get connected
    gcs.AddPathLengthCost()
    opts = GraphOfConvexSetsOptions()
    traj, result = gcs.SolvePath(s, t, opts)
    print("success:", result.is_success(), "solver:", result.get_solver_id().name())
    if result.is_success():
        ts = np.linspace(traj.start_time(), traj.end_time(), 400)
        P = np.array([traj.value(x).ravel() for x in ts])
        free = bool(np.all(np.abs(P).sum(1) >= 1 - 1e-6))
        print(f"  path length {np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)):.4f} (exact shortest {2*np.hypot(1.5, 0.6):.4f}), all 400 samples collision-free: {free}")
        print("  passes near the top corner (0, 1):", np.round(P[np.argmin(np.abs(P[:, 0]))], 3))


if __name__ == "__main__":
    {"ik": run_ik, "gcs": run_gcs}[sys.argv[1]]()
