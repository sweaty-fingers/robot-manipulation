from torch import Tensor, nn

from robot_manipulation.encoders.base import Encoder


class ProprioEncoder(Encoder):
    """Small MLP on raw proprioceptive state. The only concrete encoder in
    Phase 0 — vision/audio/tactile wrap pretrained backbones later behind the
    same Encoder interface."""

    def __init__(self, input_dim: int, output_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(input_dim, output_dim), nn.ReLU())
        self.output_dim = output_dim

    def forward(self, x: Tensor) -> Tensor:
        return self.net(x)
