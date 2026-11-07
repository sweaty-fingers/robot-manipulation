import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import PrismaticJoint, SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from robot_manipulation.envs.base import ActionSpec, ObservationSpec, RobotEnv

_LINK_INERTIA = SpatialInertia(1e-6, [0, 0, 0], UnitInertia(1e-9, 1e-9, 1e-9))


class DrakeEnv(RobotEnv):
    """3-DOF point-mass reach task. No URDF/SDF asset — the plant is three
    chained prismatic joints (x, y, z) built directly via the Drake API, which
    is enough to exercise the full env<->data<->train seam before any real
    robot model is introduced."""

    def __init__(
        self,
        control_dt: float = 0.02,
        target: np.ndarray | None = None,
        success_radius: float = 0.05,
        max_force: float = 5.0,
        max_episode_steps: int = 200,
    ):
        self.control_dt = control_dt
        self.target = np.asarray(target if target is not None else [0.5, 0.3, 0.2], dtype=np.float64)
        self.success_radius = success_radius
        self.max_episode_steps = max_episode_steps

        self.plant = MultibodyPlant(time_step=control_dt)
        slider_x = self.plant.AddRigidBody("slider_x", _LINK_INERTIA)
        slider_y = self.plant.AddRigidBody("slider_y", _LINK_INERTIA)
        ball_inertia = SpatialInertia(1.0, [0, 0, 0], UnitInertia(1e-3, 1e-3, 1e-3))
        ball = self.plant.AddRigidBody("ball", ball_inertia)

        joint_x = self.plant.AddJoint(
            PrismaticJoint("joint_x", self.plant.world_frame(), slider_x.body_frame(), [1.0, 0.0, 0.0])
        )
        joint_y = self.plant.AddJoint(
            PrismaticJoint("joint_y", slider_x.body_frame(), slider_y.body_frame(), [0.0, 1.0, 0.0])
        )
        joint_z = self.plant.AddJoint(
            PrismaticJoint("joint_z", slider_y.body_frame(), ball.body_frame(), [0.0, 0.0, 1.0])
        )
        for joint in (joint_x, joint_y, joint_z):
            self.plant.AddJointActuator(f"{joint.name()}_actuator", joint)
        self.plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
        self.plant.Finalize()

        builder = DiagramBuilder()
        builder.AddSystem(self.plant)
        self.diagram = builder.Build()
        self.simulator = Simulator(self.diagram)
        self._actuation_port = self.plant.get_actuation_input_port()
        self._ball_body = ball

        self.observation_spec = ObservationSpec(proprio_dim=self.plant.num_positions() + self.plant.num_velocities())
        self.action_spec = ActionSpec(
            dim=self.plant.num_actuators(),
            low=np.full(self.plant.num_actuators(), -max_force),
            high=np.full(self.plant.num_actuators(), max_force),
        )
        self._step_count = 0

    def reset(self, seed: int | None = None) -> dict[str, np.ndarray]:
        rng = np.random.default_rng(seed)
        self.simulator.get_mutable_context().SetTime(0.0)
        plant_context = self.plant.GetMyMutableContextFromRoot(self.simulator.get_mutable_context())
        start = rng.uniform(-0.3, 0.3, size=3)
        self.plant.SetPositions(plant_context, start)
        self.plant.SetVelocities(plant_context, np.zeros(3))
        self.simulator.Initialize()
        self._step_count = 0
        return self._get_obs(plant_context)

    def step(self, action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, dict]:
        action = np.clip(np.asarray(action, dtype=np.float64), self.action_spec.low, self.action_spec.high)
        context = self.simulator.get_mutable_context()
        self._actuation_port.FixValue(self.plant.GetMyMutableContextFromRoot(context), action)
        self.simulator.AdvanceTo(context.get_time() + self.control_dt)
        plant_context = self.plant.GetMyMutableContextFromRoot(context)
        obs = self._get_obs(plant_context)
        position = obs["proprio"][:3]
        distance = float(np.linalg.norm(position - self.target))
        success = distance < self.success_radius
        self._step_count += 1
        done = success or self._step_count >= self.max_episode_steps
        reward = -distance
        return obs, reward, done, {"success": success, "distance": distance}

    def close(self) -> None:
        pass

    def _get_obs(self, plant_context) -> dict[str, np.ndarray]:
        positions = self.plant.GetPositions(plant_context)
        velocities = self.plant.GetVelocities(plant_context)
        return {"proprio": np.concatenate([positions, velocities]).astype(np.float32)}


if __name__ == "__main__":
    env = DrakeEnv()
    obs = env.reset(seed=0)
    print("reset obs:", obs)
    for _ in range(5):
        action = np.random.uniform(-1, 1, size=env.action_spec.dim)
        obs, reward, done, info = env.step(action)
        print(obs, reward, done, info)
