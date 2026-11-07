import numpy as np
import torch
from torch.utils.data import Dataset

from robot_manipulation.data.dataset_builder import list_episodes, load_episode


class ManipulationDataset(Dataset):
    """Flattens all (proprio_t, action_t) steps across episodes in data_dir
    into one indexable dataset for behavior cloning."""

    def __init__(self, data_dir: str):
        proprios, actions = [], []
        for path in list_episodes(data_dir):
            episode = load_episode(path)
            proprios.append(episode.proprio)
            actions.append(episode.action)
        if not proprios:
            raise ValueError(f"no episodes found in {data_dir}")
        self.proprio = np.concatenate(proprios, axis=0).astype(np.float32)
        self.action = np.concatenate(actions, axis=0).astype(np.float32)

    def __len__(self) -> int:
        return len(self.proprio)

    def __getitem__(self, idx: int) -> tuple[dict[str, torch.Tensor], torch.Tensor]:
        obs = {"proprio": torch.from_numpy(self.proprio[idx])}
        return obs, torch.from_numpy(self.action[idx])
