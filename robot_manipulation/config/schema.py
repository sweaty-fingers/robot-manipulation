from dataclasses import dataclass, field


@dataclass
class EncoderSetConfig:
    proprio_output_dim: int = 64
    fusion_output_dim: int = 128


@dataclass
class ActionExpertConfig:
    type: str = "mlp"
    hidden_dims: tuple[int, ...] = (256, 256)


@dataclass
class EnvConfig:
    backend: str = "drake"
    control_dt: float = 0.02
    target: tuple[float, float, float] = (0.5, 0.3, 0.2)
    success_radius: float = 0.05
    max_force: float = 5.0
    max_episode_steps: int = 200
    # Backend-specific kwargs for envs that don't use the reach-task fields above.
    params: dict = field(default_factory=dict)


@dataclass
class TrainConfig:
    epochs: int = 50
    batch_size: int = 64
    lr: float = 1e-3
    seed: int = 0


@dataclass
class ExperimentConfig:
    name: str = "skeleton"
    encoders: EncoderSetConfig = field(default_factory=EncoderSetConfig)
    action_expert: ActionExpertConfig = field(default_factory=ActionExpertConfig)
    env: EnvConfig = field(default_factory=EnvConfig)
    train: TrainConfig = field(default_factory=TrainConfig)
    data_dir: str = "data/episodes/skeleton"
    checkpoint_dir: str = "runs/skeleton"
