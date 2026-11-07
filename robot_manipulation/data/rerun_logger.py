import numpy as np
import rerun as rr

# Fixed entity-path schema so dataset_builder can rely on it instead of
# re-discovering structure per episode.
PROPRIO_PATH = "robot/proprio"
ACTION_PATH = "action"
SUCCESS_PATH = "episode/success"


class RerunLogger:
    def __init__(self, episode_id: str, out_dir: str = "data/rrd", spawn_viewer: bool = False):
        rr.init(f"episode_{episode_id}", spawn=spawn_viewer)
        self.out_path = f"{out_dir}/{episode_id}.rrd"

    def log_step(self, t: int, obs: dict[str, np.ndarray], action: np.ndarray) -> None:
        rr.set_time("step", sequence=t)
        rr.log(PROPRIO_PATH, rr.Tensor(obs["proprio"]))
        rr.log(ACTION_PATH, rr.Tensor(action))

    def finalize(self, success: bool) -> str:
        rr.log(SUCCESS_PATH, rr.Scalars(float(success)), static=True)
        rr.save(self.out_path)
        return self.out_path
