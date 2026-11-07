import argparse

import numpy as np

from robot_manipulation.config.loader import load_experiment_config
from robot_manipulation.data.dataset_builder import Episode, save_episode
from robot_manipulation.data.rerun_logger import RerunLogger
from robot_manipulation.factory import build_env


def scripted_action(obs: np.ndarray, target: np.ndarray, kp: float = 4.0, kd: float = 2.0) -> np.ndarray:
    position, velocity = obs[:3], obs[3:]
    return kp * (target - position) - kd * velocity


def collect(config_path: str, n_episodes: int, out_dir: str, rrd_dir: str) -> None:
    config = load_experiment_config(config_path)
    env = build_env(config.env)
    target = np.array(config.env.target)

    successes = 0
    for episode_idx in range(n_episodes):
        episode_id = f"{config.name}_{episode_idx:04d}"
        logger = RerunLogger(episode_id, out_dir=rrd_dir)
        obs = env.reset(seed=episode_idx)
        proprio_steps, action_steps = [], []
        done = False
        while not done:
            action = scripted_action(obs["proprio"], target)
            logger.log_step(len(proprio_steps), obs, action)
            proprio_steps.append(obs["proprio"])
            action_steps.append(action.astype(np.float32))
            obs, _, done, info = env.step(action)
        logger.finalize(info["success"])
        successes += int(info["success"])

        episode = Episode(
            proprio=np.stack(proprio_steps), action=np.stack(action_steps), success=info["success"]
        )
        save_episode(episode, out_dir, episode_id)

    print(f"collected {n_episodes} episodes, success rate {successes / n_episodes:.2f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/experiments/00_skeleton.yaml")
    parser.add_argument("--n-episodes", type=int, default=50)
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--rrd-dir", default="data/rrd")
    args = parser.parse_args()

    config = load_experiment_config(args.config)
    collect(args.config, args.n_episodes, args.out_dir or config.data_dir, args.rrd_dir)
