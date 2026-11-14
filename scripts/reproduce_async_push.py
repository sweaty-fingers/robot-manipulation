"""Phase B baseline: show that asynchronous finger contact pushes the object.

Closes three fingers at constant speed with per-finger start delays and prints
how far the object's centre moves. Zero spread = synchronous (ideal) closing.
"""
import numpy as np

from robot_manipulation.envs.grasp_env import ThreeFingerGraspEnv


def close_fingers(spread: float, speed: float = 0.08, floor: float = 0.02, seed: int = 0) -> dict:
    env = ThreeFingerGraspEnv()
    env.reset(seed=seed)
    delays = np.array([0.0, 0.5, 1.0]) * spread
    for k in range(env.max_episode_steps):
        t = k * env.control_dt
        target = np.maximum(floor, env.finger_start - speed * np.maximum(0.0, t - delays))
        _, _, done, info = env.step(target)
    return info


if __name__ == "__main__":
    print(f"{'spread (s)':>10} | {'object displacement (mm)':>24} | all 3 in contact")
    for spread in (0.0, 0.1, 0.2, 0.4, 0.6, 1.0):
        info = close_fingers(spread)
        print(f"{spread:>10.2f} | {info['object_displacement'] * 1000:>24.2f} | {bool(info['success'])}")
