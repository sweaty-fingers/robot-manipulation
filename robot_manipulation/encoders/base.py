from abc import ABC, abstractmethod

from torch import Tensor, nn


class Encoder(nn.Module, ABC):
    """Maps a modality-specific raw tensor to a fixed-size embedding."""

    output_dim: int

    @abstractmethod
    def forward(self, x: Tensor) -> Tensor: ...

    def freeze(self) -> None:
        for p in self.parameters():
            p.requires_grad_(False)
