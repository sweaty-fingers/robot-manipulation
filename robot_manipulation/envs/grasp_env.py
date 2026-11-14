import numpy as np
from pydrake.geometry import Box, Cylinder, Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.tree import (
    PdControllerGains,
    PrismaticJoint,
    SpatialInertia,
    UnitInertia,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from robot_manipulation.envs.base import ActionSpec, ObservationSpec, RobotEnv

_FINGER_INERTIA = SpatialInertia(0.05, [0, 0, 0], UnitInertia(1e-5, 1e-5, 1e-5))
_NUM_FINGERS = 3


class ThreeFingerGraspEnv(RobotEnv):
    """Phase B: minimal 3-finger pinch of a cylinder resting on a table.

    Each finger is one position-controlled prismatic DOF sliding radially in
    the table plane (fingers 120 deg apart). That is the smallest model in
    which *asynchronous contact* exists: whichever finger touches first pushes
    the free object toward the others before they close. The object's ground
    truth displacement is reported in `info` (never in obs — obs is
    proprioception only, per the project's vision+proprio decision).

    Action: absolute radial target (m) for each finger tip centre.
    """

    def __init__(
        self,
        control_dt: float = 0.02,
        sim_dt: float = 1e-3,
        object_radius: float = 0.03,
        object_height: float = 0.06,
        object_mass: float = 0.1,
        table_friction: float = 0.4,
        finger_friction: float = 0.8,
        finger_start: float = 0.10,
        finger_min: float = 0.0,
        finger_max: float = 0.15,
        kp: float = 300.0,
        kd: float = 15.0,
        max_episode_steps: int = 150,
        object_pos_noise: float = 0.0,
    ):
        self.control_dt = control_dt
        self.object_radius = object_radius
        self.finger_start = finger_start
        self.max_episode_steps = max_episode_steps
        self.object_pos_noise = object_pos_noise
        self._tip_radius = 0.01
        self._rest_z = object_height / 2

        builder = DiagramBuilder()
        plant, _scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_dt)
        self.plant = plant
        self._directions = []
        self._finger_bodies = []
        for i in range(_NUM_FINGERS):
            angle = 2 * np.pi * i / _NUM_FINGERS
            d = np.array([np.cos(angle), np.sin(angle), 0.0])
            body = plant.AddRigidBody(f"finger_{i}", _FINGER_INERTIA)
            joint = plant.AddJoint(
                PrismaticJoint(
                    f"finger_{i}_slide", plant.world_frame(), body.body_frame(), d, finger_min, finger_max
                )
            )
            actuator = plant.AddJointActuator(f"finger_{i}_act", joint)
            actuator.set_controller_gains(PdControllerGains(p=kp, d=kd))
            plant.RegisterCollisionGeometry(
                body, RigidTransform([0, 0, 0.02]), Sphere(self._tip_radius), f"finger_{i}_tip",
                CoulombFriction(finger_friction, finger_friction),
            )
            self._directions.append(d)
            self._finger_bodies.append(body)

        self._object = plant.AddRigidBody(
            "object",
            SpatialInertia.SolidCylinderWithMass(object_mass, object_radius, object_height, [0, 0, 1]),
        )
        plant.RegisterCollisionGeometry(
            self._object, RigidTransform(), Cylinder(object_radius, object_height), "object_col",
            CoulombFriction(table_friction, table_friction),
        )
        plant.RegisterCollisionGeometry(
            plant.world_body(), RigidTransform([0, 0, -0.05]), Box(2, 2, 0.1), "table",
            CoulombFriction(table_friction, table_friction),
        )
        plant.Finalize()

        self.diagram = builder.Build()
        self.simulator = Simulator(self.diagram)
        self._desired_port = plant.get_desired_state_input_port(self._finger_bodies[0].model_instance())
        self._actuation_port = plant.get_actuation_input_port()

        n = _NUM_FINGERS
        self.observation_spec = ObservationSpec(proprio_dim=2 * n)
        self.action_spec = ActionSpec(dim=n, low=np.full(n, finger_min), high=np.full(n, finger_max))
        self._step_count = 0
        self._object_start = np.zeros(3)

    # --- helpers -----------------------------------------------------------
    def _plant_context(self):
        return self.plant.GetMyMutableContextFromRoot(self.simulator.get_mutable_context())

    def _finger_state(self, ctx):
        q, qd = [], []
        for body in self._finger_bodies:
            joint = self.plant.GetJointByName(f"{body.name()}_slide")
            q.append(joint.get_translation(ctx))
            qd.append(joint.get_translation_rate(ctx))
        return np.array(q), np.array(qd)

    def object_position(self) -> np.ndarray:
        ctx = self.plant.GetMyContextFromRoot(self.simulator.get_context())
        return self.plant.EvalBodyPoseInWorld(ctx, self._object).translation().copy()

    def contact_flags(self) -> np.ndarray:
        """Which fingers currently touch the object (ground truth, for metrics)."""
        ctx = self.plant.GetMyContextFromRoot(self.simulator.get_context())
        results = self.plant.get_contact_results_output_port().Eval(ctx)
        flags = np.zeros(_NUM_FINGERS, dtype=bool)
        for k in range(results.num_point_pair_contacts()):
            info = results.point_pair_contact_info(k)
            names = {
                self.plant.get_body(info.bodyA_index()).name(),
                self.plant.get_body(info.bodyB_index()).name(),
            }
            if "object" in names:
                for i in range(_NUM_FINGERS):
                    if f"finger_{i}" in names:
                        flags[i] = True
        return flags

    # --- RobotEnv ----------------------------------------------------------
    def reset(self, seed: int | None = None) -> dict[str, np.ndarray]:
        rng = np.random.default_rng(seed)
        context = self.simulator.get_mutable_context()
        context.SetTime(0.0)
        ctx = self._plant_context()
        for body in self._finger_bodies:
            self.plant.GetJointByName(f"{body.name()}_slide").set_translation(ctx, self.finger_start)
            self.plant.GetJointByName(f"{body.name()}_slide").set_translation_rate(ctx, 0.0)
        xy = rng.uniform(-1, 1, 2) * self.object_pos_noise
        self.plant.SetFreeBodyPose(ctx, self._object, RigidTransform([xy[0], xy[1], self._rest_z + 1e-4]))
        self.plant.SetFreeBodySpatialVelocity(ctx, self._object, SpatialVelocity(np.zeros(6)))
        self._set_targets(np.full(_NUM_FINGERS, self.finger_start))
        self.simulator.Initialize()
        self._step_count = 0
        self._object_start = self.object_position()
        return self._get_obs()

    def _set_targets(self, q_des: np.ndarray) -> None:
        ctx = self._plant_context()
        self._desired_port.FixValue(ctx, np.concatenate([q_des, np.zeros(_NUM_FINGERS)]))
        self._actuation_port.FixValue(ctx, np.zeros(_NUM_FINGERS))

    def step(self, action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, dict]:
        q_des = np.clip(np.asarray(action, dtype=np.float64), self.action_spec.low, self.action_spec.high)
        self._set_targets(q_des)
        self.simulator.AdvanceTo(self.simulator.get_context().get_time() + self.control_dt)
        self._step_count += 1
        obs = self._get_obs()
        disp = float(np.linalg.norm(self.object_position()[:2] - self._object_start[:2]))
        contacts = self.contact_flags()
        closed = bool(contacts.all())
        done = self._step_count >= self.max_episode_steps
        info = {
            "success": closed,
            "object_displacement": disp,
            "contacts": contacts,
            "object_pos": self.object_position(),
        }
        return obs, -disp, done, info

    def close(self) -> None:
        pass

    def _get_obs(self) -> dict[str, np.ndarray]:
        q, qd = self._finger_state(self._plant_context())
        return {"proprio": np.concatenate([q, qd]).astype(np.float32)}
