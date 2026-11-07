from omegaconf import OmegaConf

from robot_manipulation.config.schema import ExperimentConfig


def load_experiment_config(path: str) -> ExperimentConfig:
    schema = OmegaConf.structured(ExperimentConfig)
    loaded = OmegaConf.load(path)
    merged = OmegaConf.merge(schema, loaded)
    return OmegaConf.to_object(merged)
