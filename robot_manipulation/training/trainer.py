from pathlib import Path

import torch
from torch.utils.data import DataLoader

from robot_manipulation.config.schema import TrainConfig
from robot_manipulation.policies.policy import Policy


class Trainer:
    def __init__(self, config: TrainConfig):
        self.config = config

    def fit(self, policy: Policy, dataset, checkpoint_dir: str) -> None:
        torch.manual_seed(self.config.seed)
        loader = DataLoader(dataset, batch_size=self.config.batch_size, shuffle=True)
        optimizer = torch.optim.Adam(policy.parameters(), lr=self.config.lr)

        for epoch in range(self.config.epochs):
            total_loss = 0.0
            for obs, action in loader:
                loss = policy.compute_loss(obs, action)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                total_loss += loss.item()
            if epoch % max(1, self.config.epochs // 10) == 0 or epoch == self.config.epochs - 1:
                print(f"epoch {epoch}: loss={total_loss / len(loader):.4f}")

        Path(checkpoint_dir).mkdir(parents=True, exist_ok=True)
        torch.save(policy.state_dict(), f"{checkpoint_dir}/policy.pt")
