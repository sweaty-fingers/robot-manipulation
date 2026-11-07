from dataclasses import dataclass
from pathlib import Path

import numpy as np

# rerun-sdk (0.42) ships no public read-back API for .rrd files — only the
# write path exists. So rerun is used purely for live/visual logging
# (debugging rollouts in the viewer), and the actual training dataset is
# captured directly from rollout arrays at collection time, saved as one .npz
# per episode here. Each episode's .npz and .rrd share the same episode_id so
# they can still be cross-referenced for debugging.


@dataclass
class Episode:
    proprio: np.ndarray  # (T, proprio_dim)
    action: np.ndarray  # (T, action_dim)
    success: bool


def save_episode(episode: Episode, out_dir: str, episode_id: str) -> str:
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    path = f"{out_dir}/{episode_id}.npz"
    np.savez(path, proprio=episode.proprio, action=episode.action, success=episode.success)
    return path


def load_episode(path: str) -> Episode:
    data = np.load(path)
    return Episode(proprio=data["proprio"], action=data["action"], success=bool(data["success"]))


def list_episodes(data_dir: str) -> list[str]:
    return sorted(str(p) for p in Path(data_dir).glob("*.npz"))
