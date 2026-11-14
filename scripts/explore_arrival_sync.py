"""Why touch isn't strictly required: synchronise arrival *before* contact.

Object is offset from where the controller assumes it is. Compare:
  blind    - all fingers close at the same speed toward the assumed centre
  geometry - per-finger speed set so every tip reaches the object surface at the
             same time, using the (vision-estimated) object offset
  geometry+err - same, but the vision estimate has a position error

Metric: object displacement (mm) when all three fingers are in contact.
Tip-to-surface distance uses fingertip radial position from proprioception
(forward kinematics) and object position from "vision" (here: ground truth + noise).
"""
import numpy as np

from robot_manipulation.envs.grasp_env import ThreeFingerGraspEnv

TIP, R = 0.01, 0.03


def run(offset, speed, mode, vision_err=0.0, kp=300.0, seed=0):
    env = ThreeFingerGraspEnv(kp=kp, kd=15.0 * (kp / 300.0) ** 0.5, max_episode_steps=200, object_pos_noise=0.0)
    env.reset(seed=seed)
    # move object to the unknown offset (x direction), keep it at rest
    from pydrake.math import RigidTransform
    ctx = env._plant_context()
    env.plant.SetFreeBodyPose(ctx, env._object, RigidTransform([offset, 0, env._rest_z + 1e-4]))
    env._object_start = env.object_position()
    est = np.array([offset + vision_err, 0.0]) if mode != "blind" else np.zeros(2)
    dirs = np.array([d[:2] for d in env._directions])
    contact_q = dirs @ est + R + TIP                    # radial position where tip touches surface
    dist = env.finger_start - contact_q                 # distance still to travel for each finger
    t_arrive = dist.max() / speed if mode == "blind" else dist.max() / speed
    # blind: identical speed -> arrival times differ; geometry: speeds scaled so all arrive at t_arrive
    for k in range(env.max_episode_steps):
        t = k * env.control_dt
        if mode == "blind":
            tgt = np.maximum(0.02, env.finger_start - speed * t)
        else:
            tgt = np.maximum(0.02, env.finger_start - (dist / t_arrive) * t)
        _, _, _, info = env.step(tgt)
    return info["object_displacement"] * 1000, bool(info["success"])


if __name__ == "__main__":
    print(f"{'offset(mm)':>10} {'speed':>6} | {'blind':>8} {'geometry':>9} {'geom,err2mm':>12}   (object displacement, mm)")
    for offset in (0.0, 0.005, 0.010):
        for speed in (0.04, 0.08, 0.16):
            row = [run(offset, speed, m, e)[0] for m, e in (("blind", 0), ("geometry", 0), ("geometry", 0.002))]
            print(f"{offset*1000:>10.0f} {speed:>6.2f} | {row[0]:>8.2f} {row[1]:>9.2f} {row[2]:>12.2f}")
