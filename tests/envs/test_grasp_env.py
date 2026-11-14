import numpy as np

from robot_manipulation.envs.grasp_env import ThreeFingerGraspEnv


def _close(spread):
    env = ThreeFingerGraspEnv()
    env.reset(seed=0)
    delays = np.array([0.0, 0.5, 1.0]) * spread
    for k in range(env.max_episode_steps):
        t = k * env.control_dt
        _, _, _, info = env.step(np.maximum(0.02, env.finger_start - 0.08 * np.maximum(0.0, t - delays)))
    return info


def test_reset_and_step_contract():
    env = ThreeFingerGraspEnv(max_episode_steps=3)
    obs = env.reset(seed=0)
    assert obs["proprio"].shape == (env.observation_spec.proprio_dim,)
    obs, reward, done, info = env.step(np.full(env.action_spec.dim, 0.1))
    assert isinstance(reward, float) and isinstance(done, bool)
    assert {"success", "object_displacement"} <= info.keys()


def test_async_contact_pushes_object_more_than_sync():
    sync, async_ = _close(0.0), _close(1.0)
    assert sync["success"] and async_["success"]
    assert async_["object_displacement"] > 5 * max(sync["object_displacement"], 1e-3)
