import numpy as np

from robot_manipulation.envs.drake_env import DrakeEnv


def test_reset_returns_proprio_matching_spec():
    env = DrakeEnv()
    obs = env.reset(seed=0)
    assert obs["proprio"].shape == (env.observation_spec.proprio_dim,)


def test_step_contract():
    env = DrakeEnv(max_episode_steps=5)
    obs = env.reset(seed=0)
    action = np.zeros(env.action_spec.dim, dtype=np.float32)
    obs, reward, done, info = env.step(action)
    assert obs["proprio"].shape == (env.observation_spec.proprio_dim,)
    assert isinstance(reward, float)
    assert isinstance(done, bool)
    assert "success" in info


def test_episode_terminates_within_max_steps():
    env = DrakeEnv(max_episode_steps=10)
    env.reset(seed=0)
    action = np.zeros(env.action_spec.dim, dtype=np.float32)
    for _ in range(10):
        _, _, done, _ = env.step(action)
    assert done
