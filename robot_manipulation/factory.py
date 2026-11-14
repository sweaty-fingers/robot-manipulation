from robot_manipulation.action_experts.base import ActionExpert
from robot_manipulation.action_experts.mlp import MLPActionExpert
from robot_manipulation.config.schema import ActionExpertConfig, EncoderSetConfig, EnvConfig
from robot_manipulation.encoders.fusion import ObservationEncoder
from robot_manipulation.encoders.proprioception import ProprioEncoder
from robot_manipulation.envs.base import RobotEnv
from robot_manipulation.envs.drake_env import DrakeEnv
from robot_manipulation.envs.grasp_env import ThreeFingerGraspEnv

# Single lookup point for each swappable piece. Adding a new env backend,
# encoder, or action-expert architecture means adding one entry here plus its
# module — scripts/train.py and scripts/evaluate.py never change.
_ENV_BACKENDS = {"drake": DrakeEnv, "drake_grasp": ThreeFingerGraspEnv}
_ACTION_EXPERTS = {"mlp": MLPActionExpert}


def build_env(config: EnvConfig) -> RobotEnv:
    cls = _ENV_BACKENDS[config.backend]
    if config.backend == "drake_grasp":
        return cls(control_dt=config.control_dt, max_episode_steps=config.max_episode_steps, **config.params)
    return cls(
        control_dt=config.control_dt,
        target=list(config.target),
        success_radius=config.success_radius,
        max_force=config.max_force,
        max_episode_steps=config.max_episode_steps,
    )


def build_observation_encoder(config: EncoderSetConfig, proprio_dim: int) -> ObservationEncoder:
    proprio_encoder = ProprioEncoder(input_dim=proprio_dim, output_dim=config.proprio_output_dim)
    return ObservationEncoder({"proprio": proprio_encoder}, output_dim=config.fusion_output_dim)


def build_action_expert(config: ActionExpertConfig, input_dim: int, action_dim: int) -> ActionExpert:
    cls = _ACTION_EXPERTS[config.type]
    return cls(input_dim=input_dim, action_dim=action_dim, hidden_dims=config.hidden_dims)
