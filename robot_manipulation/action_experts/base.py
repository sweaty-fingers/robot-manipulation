from abc import ABC, abstractmethod

from torch import Tensor, nn


class ActionExpert(nn.Module, ABC):
    """Consumes the fused observation embedding and produces an action.
    Trainer/Policy call only predict/compute_loss, never branch on concrete
    type, so swapping architectures (MLP -> diffusion -> transformer) is a
    config change, not a call-site change."""

    action_dim: int

    @abstractmethod
    def predict(self, obs_embed: Tensor) -> Tensor: ...

    @abstractmethod
    def compute_loss(self, obs_embed: Tensor, target_action: Tensor) -> Tensor: ...
