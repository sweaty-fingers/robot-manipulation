from torch import Tensor, nn
from torch.nn import functional as F

from robot_manipulation.action_experts.base import ActionExpert


class MLPActionExpert(ActionExpert):
    def __init__(self, input_dim: int, action_dim: int, hidden_dims: tuple[int, ...] = (256, 256)):
        super().__init__()
        layers: list[nn.Module] = []
        d = input_dim
        for h in hidden_dims:
            layers += [nn.Linear(d, h), nn.ReLU()]
            d = h
        layers.append(nn.Linear(d, action_dim))
        self.net = nn.Sequential(*layers)
        self.action_dim = action_dim

    def predict(self, obs_embed: Tensor) -> Tensor:
        return self.net(obs_embed)

    def compute_loss(self, obs_embed: Tensor, target_action: Tensor) -> Tensor:
        return F.mse_loss(self.predict(obs_embed), target_action)
