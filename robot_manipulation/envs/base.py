from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class ObservationSpec:
    proprio_dim: int
    rgb_shape: tuple[int, int, int] | None = None
    tactile_shape: tuple[int, ...] | None = None
    audio_sample_rate: int | None = None
    has_contact_forces: bool = False


@dataclass
class ActionSpec:
    dim: int
    low: np.ndarray
    high: np.ndarray
    is_delta: bool = False


class RobotEnv(ABC):
    """Backend-agnostic manipulation env contract. Obs dict keys: "proprio" is
    always present; "rgb"/"tactile"/"audio"/"contact_forces" are present only
    when observation_spec marks them as available."""

    observation_spec: ObservationSpec
    action_spec: ActionSpec

    @abstractmethod
    def reset(self, seed: int | None = None) -> dict[str, np.ndarray]: ...

    @abstractmethod
    def step(self, action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, dict]:
        """Returns (obs, reward, done, info); info includes at least {"success": bool}."""

    @abstractmethod
    def close(self) -> None: ...
