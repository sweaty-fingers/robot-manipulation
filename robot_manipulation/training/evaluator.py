import numpy as np
import torch

from robot_manipulation.envs.base import RobotEnv
from robot_manipulation.policies.policy import Policy


class Evaluator:
    def __init__(self, env: RobotEnv, n_episodes: int = 20):
        self.env = env
        self.n_episodes = n_episodes

    def run(self, policy: Policy) -> dict:
        policy.eval()
        successes = 0
        for episode in range(self.n_episodes):
            obs = self.env.reset(seed=1000 + episode)
            done = False
            while not done:
                with torch.no_grad():
                    obs_t = {"proprio": torch.from_numpy(obs["proprio"]).unsqueeze(0)}
                    action = policy.predict(obs_t).squeeze(0).numpy()
                obs, _, done, info = self.env.step(action)
            successes += int(info["success"])
        return {"success_rate": successes / self.n_episodes}
