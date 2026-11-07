from torch import Tensor, nn

from robot_manipulation.action_experts.base import ActionExpert
from robot_manipulation.encoders.fusion import ObservationEncoder


class Policy(nn.Module):
    def __init__(self, observation_encoder: ObservationEncoder, action_expert: ActionExpert):
        super().__init__()
        self.observation_encoder = observation_encoder
        self.action_expert = action_expert

    def predict(self, obs: dict[str, Tensor]) -> Tensor:
        return self.action_expert.predict(self.observation_encoder(obs))

    def compute_loss(self, obs: dict[str, Tensor], target_action: Tensor) -> Tensor:
        return self.action_expert.compute_loss(self.observation_encoder(obs), target_action)
