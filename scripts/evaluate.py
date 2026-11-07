import argparse

import torch

from robot_manipulation.config.loader import load_experiment_config
from robot_manipulation.factory import build_action_expert, build_env, build_observation_encoder
from robot_manipulation.policies.policy import Policy
from robot_manipulation.training.evaluator import Evaluator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/experiments/00_skeleton.yaml")
    parser.add_argument("--n-episodes", type=int, default=20)
    args = parser.parse_args()

    config = load_experiment_config(args.config)
    env = build_env(config.env)

    observation_encoder = build_observation_encoder(config.encoders, env.observation_spec.proprio_dim)
    action_expert = build_action_expert(
        config.action_expert, input_dim=observation_encoder.output_dim, action_dim=env.action_spec.dim
    )
    policy = Policy(observation_encoder, action_expert)
    policy.load_state_dict(torch.load(f"{config.checkpoint_dir}/policy.pt"))

    result = Evaluator(env, n_episodes=args.n_episodes).run(policy)
    print(result)
