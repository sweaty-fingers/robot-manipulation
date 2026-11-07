import torch
from torch import Tensor, nn

from robot_manipulation.encoders.base import Encoder


class ObservationEncoder(nn.Module):
    """Fuses whatever modality keys are present in the obs dict (vision/audio/
    tactile, each optional) into one vector. Adding a modality later is a
    matter of passing another Encoder here — this module's call sites never
    change."""

    def __init__(self, encoders: dict[str, Encoder], output_dim: int = 128):
        super().__init__()
        self.encoders = nn.ModuleDict(encoders)
        total_in = sum(e.output_dim for e in encoders.values())
        self.fuse = nn.Sequential(nn.Linear(total_in, output_dim), nn.ReLU())
        self.output_dim = output_dim

    def forward(self, obs: dict[str, Tensor]) -> Tensor:
        parts = [self.encoders[k](obs[k]) for k in self.encoders if k in obs]
        return self.fuse(torch.cat(parts, dim=-1))
