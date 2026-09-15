from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R


DEFAULT_INIT_DEG = [0.0, 99.88, -91.26, 0.0, 63.84, 0.0]
DEFAULT_JOINT_LIMITS_DEG = [
    (-150.0, 150.0),
    (0.0, 180.0),
    (-170.0, 0.0),
    (-100.0, 100.0),
    (-70.0, 70.0),
    (-120.0, 120.0),
]

# ── Flat-grasp IK constants ──────────────────────────────────────────────
# Ported verbatim from roboarm's arm/piper_ctrl_by_sdk_flat_hand.py, which is
# the authority for these numbers — they were tuned on the real arm, and the
# class docstring there records the resulting quality (3.87 deg @ 3.3 mm at
# the corridor centre, 2.17 deg @ 1.9 mm at z=0.16, over 40 Pareto starts).
DEFAULT_FLAT_TEACH_JOINTS_DEG = [-3.867, 90.286, -7.659, 0.764, -74.85, 6.748]
# 1-BASED joint numbers, matching roboarm's linker_hand_pin_joints config.
# These are the wrist joints the flat grasp holds at their taught angles while
# the shoulder/elbow (J2/J3) absorb the reach.
DEFAULT_PIN_JOINTS = [[5, -69.0], [6, 6.75]]
DEFAULT_FLAT_POS_WEIGHT_M = 0.008
DEFAULT_FLAT_ROT_TOL_DEG = 5.0
DEFAULT_FLAT_MULTISTART = 10
# Cost scale: cost = (pos_err/0.008)^2 + (rot_err/5deg)^2, so a perfect flat
# pose scores ~0.8 and the terms are commensurate — 1 unit of cost is roughly
# "8 mm of position error OR 5 deg of attitude error". 4.0 therefore admits
# about 1.6 cm combined, well inside a flat-palm grasp's needs.
DEFAULT_FLAT_SUCCESS_COST = 4.0


@dataclass(frozen=True)
class IkResult:
    success: bool
    joints_rad: np.ndarray
    cost: float
    message: str
    # Achieved errors at the returned solution. Carried on the result rather
    # than recomputed by callers because the cost alone is not interpretable —
    # a cost of 3.0 could be 1.4 cm of position or 8.6 deg of attitude, and an
    # operator staring at "ik_failed cost=4.2" has no way to tell which.
    pos_err_m: float = 0.0
    rot_err_rad: float = 0.0

    @property
    def joints_deg(self) -> list[float]:
        return np.rad2deg(self.joints_rad).astype(float).tolist()

    @property
    def rot_err_deg(self) -> float:
        return float(np.rad2deg(self.rot_err_rad))


def default_urdf_path() -> str:
    # The body URDF. ROBOARM_URDF_PATH is what the description primitive uses;
    # PIPER_URDF_PATH is kept because older checkouts and the sim deploy used it.
    for var in ("ROBOARM_URDF_PATH", "PIPER_URDF_PATH"):
        env_path = os.environ.get(var, "")
        if env_path and Path(env_path).is_file():
            return str(Path(env_path).resolve())
    # Walk up from this file to the deploy root looking for urdf/<body>.urdf.
    # Depth-independent so it survives the vendored packages/<pkg>/ layout
    # (the old hard-coded parents[4] assumed the rbnx-boot/cache/ clone depth).
    # `piper.urdf` is the pre-2026-09-15 name, kept as a fallback so a stale
    # checkout still resolves instead of failing on the rename.
    for parent in Path(__file__).resolve().parents:
        for name in ("roboarm.urdf", "piper.urdf"):
            candidate = parent / "urdf" / name
            if candidate.is_file():
                return str(candidate.resolve())
    raise FileNotFoundError("cannot locate the body URDF; pass urdf_path explicitly")


def parse_float_list(value: Any, default: list[float]) -> list[float]:
    if value is None:
        return list(default)
    if isinstance(value, str):
        value = value.strip()
        if not value:
            return list(default)
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            value = [part.strip() for part in value.split(",")]
    return [float(v) for v in value]


def wrap_angle_rad(angle_rad: float) -> float:
    return float(np.arctan2(np.sin(angle_rad), np.cos(angle_rad)))


def pose_matrix_from_xyz_quat(
    xyz: tuple[float, float, float] | list[float] | np.ndarray,
    quat_xyzw: tuple[float, float, float, float] | list[float] | np.ndarray,
) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


def vertical_down_matrix(
    xyz: tuple[float, float, float] | list[float] | np.ndarray,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.from_euler(
        "zyx", [np.rad2deg(yaw_rad), 180.0, 0.0], degrees=True
    ).as_matrix()
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


class PiperRoboarmIk:
    """Pure IK solver ported from roboarm's PiperBySDK path."""

    position_tol_m = 0.01
    tilt_tol_rad = np.deg2rad(5.0)
    yaw_tol_rad = np.deg2rad(10.0)

    def __init__(
        self,
        urdf_path: str | None = None,
        tip_link: str = "link6",
        flat_pos_weight_m: float | None = None,
        flat_rot_tol_deg: float | None = None,
    ):
        import kinpy

        self.urdf_path = urdf_path or default_urdf_path()
        self.tip_link = tip_link
        with open(self.urdf_path, "r", encoding="utf-8") as f:
            urdf_content = f.read()
        urdf_content = re.sub(r"<\?xml[^?]*\?>", "", urdf_content, count=1)
        self.chain = kinpy.build_serial_chain_from_urdf(urdf_content, tip_link)
        self.joint_bounds = self._parse_joint_bounds(urdf_content)
        if len(self.joint_bounds) != 6:
            self.joint_bounds = [
                (np.deg2rad(lo), np.deg2rad(hi))
                for lo, hi in DEFAULT_JOINT_LIMITS_DEG
            ]
        self.flat_pos_weight_m = float(
            flat_pos_weight_m or DEFAULT_FLAT_POS_WEIGHT_M)
        self.flat_rot_tol_rad = float(np.deg2rad(
            flat_rot_tol_deg if flat_rot_tol_deg is not None
            else DEFAULT_FLAT_ROT_TOL_DEG))

    @staticmethod
    def _parse_joint_bounds(urdf_content: str) -> list[tuple[float, float]]:
        root = ElementTree.fromstring(urdf_content)
        bounds: list[tuple[float, float]] = []
        for joint_elem in root.findall("joint"):
            if joint_elem.get("type") != "revolute":
                continue
            limit = joint_elem.find("limit")
            if limit is None:
                continue
            bounds.append(
                (
                    float(limit.get("lower", "0")),
                    float(limit.get("upper", "0")),
                )
            )
            if len(bounds) == 6:
                break
        return bounds

    @classmethod
    def cost(cls, joint_angles: np.ndarray, target_pose_matrix: np.ndarray, chain) -> float:
        current_pose_matrix = chain.forward_kinematics(joint_angles).matrix()
        pos_error = np.linalg.norm(
            current_pose_matrix[:3, 3] - target_pose_matrix[:3, 3]
        )

        current_rot = R.from_matrix(current_pose_matrix[:3, :3])
        target_rot = R.from_matrix(target_pose_matrix[:3, :3])
        current_z_axis = current_rot.apply([0.0, 0.0, 1.0])
        target_z_axis = target_rot.apply([0.0, 0.0, 1.0])
        cosine = np.clip(float(np.dot(current_z_axis, target_z_axis)), -1.0, 1.0)
        tilt_error = float(np.arccos(cosine))

        current_yaw = current_rot.as_euler("zyx")[0]
        target_yaw = target_rot.as_euler("zyx")[0]
        yaw_error = abs(wrap_angle_rad(current_yaw - target_yaw))

        pos_term = (pos_error / cls.position_tol_m) ** 2
        tilt_term = (tilt_error / cls.tilt_tol_rad) ** 2
        yaw_term = (yaw_error / cls.yaw_tol_rad) ** 2
        return float(pos_term + tilt_term + yaw_term)

    def solve_pose_matrix(
        self,
        target_pose_matrix: np.ndarray,
        current_joints_rad: np.ndarray | list[float] | None = None,
        init_joints_rad: np.ndarray | list[float] | None = None,
        attempts: int = 12,
        maxiter: int = 120,
        success_cost: float = 9.0,
        seed: int = 7,
    ) -> IkResult:
        if current_joints_rad is None:
            current = np.deg2rad(DEFAULT_INIT_DEG)
        else:
            current = np.asarray(current_joints_rad, dtype=float)
        if init_joints_rad is None:
            init = np.deg2rad(DEFAULT_INIT_DEG)
        else:
            init = np.asarray(init_joints_rad, dtype=float)

        rng = np.random.default_rng(seed)
        guesses = [current, init, np.zeros(6, dtype=float)]
        span = np.array([hi - lo for lo, hi in self.joint_bounds], dtype=float)
        center = np.array([(lo + hi) * 0.5 for lo, hi in self.joint_bounds], dtype=float)
        while len(guesses) < max(1, attempts):
            if len(guesses) % 2 == 0:
                guesses.append(current + rng.normal(0.0, 0.20, 6))
            else:
                guesses.append(center + rng.uniform(-0.35, 0.35, 6) * span)

        best_joints = current
        best_cost = float("inf")
        best_msg = "no attempts"
        for guess in guesses[: max(1, attempts)]:
            x0 = np.array(
                [
                    np.clip(value, self.joint_bounds[index][0], self.joint_bounds[index][1])
                    for index, value in enumerate(guess)
                ],
                dtype=float,
            )
            result = minimize(
                self.cost,
                x0=x0,
                args=(target_pose_matrix, self.chain),
                method="SLSQP",
                bounds=self.joint_bounds,
                options={"maxiter": int(maxiter), "ftol": 1e-6, "disp": False},
            )
            cost = float(result.fun) if np.isfinite(result.fun) else float("inf")
            if cost < best_cost:
                best_cost = cost
                best_joints = np.asarray(result.x, dtype=float)
                best_msg = str(result.message)
            if result.success and best_cost <= success_cost:
                break

        return IkResult(
            success=bool(best_cost <= success_cost),
            joints_rad=best_joints,
            cost=best_cost,
            message=best_msg,
        )

    def solve_xyz_quat(
        self,
        xyz: tuple[float, float, float] | list[float] | np.ndarray,
        quat_xyzw: tuple[float, float, float, float] | list[float] | np.ndarray,
        current_joints_rad: np.ndarray | list[float] | None = None,
        **kwargs,
    ) -> IkResult:
        return self.solve_pose_matrix(
            pose_matrix_from_xyz_quat(xyz, quat_xyzw),
            current_joints_rad=current_joints_rad,
            **kwargs,
        )

    def solve_vertical_down(
        self,
        xyz: tuple[float, float, float] | list[float] | np.ndarray,
        yaw_rad: float = 0.0,
        current_joints_rad: np.ndarray | list[float] | None = None,
        **kwargs,
    ) -> IkResult:
        return self.solve_pose_matrix(
            vertical_down_matrix(xyz, yaw_rad=yaw_rad),
            current_joints_rad=current_joints_rad,
            **kwargs,
        )

    # ── Flat-palm grasp IK ───────────────────────────────────────────────

    def flat_cost(
        self, joint_angles: np.ndarray, target_pose_matrix: np.ndarray
    ) -> tuple[float, float, float]:
        """Position + FULL-ATTITUDE geodesic error, as (cost, pos_m, rot_rad).

        Why not `cost()` above: that one splits the attitude term into a
        tool-z alignment plus a ZYX yaw. That split is exact for a
        vertical-down pose, where the tool axis IS the world z and the
        remaining freedom really is yaw — but the flat grasp holds the palm
        roughly parallel to the table, i.e. the tool z is near-horizontal, and
        there "world-z yaw" stops tracking rotation about the hand's own axis.
        Roll is precisely the DOF that decides whether the palm is flat, so
        the old cost is blind to the one thing the flat grasp cares about.
        roboarm hit this too; its flat class documents the same reasoning.

        Uses `.matrix()` rather than kinpy's `Transform.rot`: kinpy stores that
        quaternion scalar-FIRST (w,x,y,z) while scipy's R.from_quat expects
        (x,y,z,w), and mixing them is off by ~165 deg with no error raised.
        roboarm fixed exactly this bug on 2026-09-11.
        """
        fk = self.chain.forward_kinematics(joint_angles).matrix()
        pos_err = float(np.linalg.norm(fk[:3, 3] - target_pose_matrix[:3, 3]))
        cur_rot = R.from_matrix(fk[:3, :3])
        tgt_rot = R.from_matrix(target_pose_matrix[:3, :3])
        rot_err = float((cur_rot.inv() * tgt_rot).magnitude())
        cost = (pos_err / self.flat_pos_weight_m) ** 2 + (
            rot_err / self.flat_rot_tol_rad
        ) ** 2
        return float(cost), pos_err, rot_err

    def _bounds_with_pins(
        self, pin_joints: list[list[float]] | None
    ) -> list[tuple[float, float]]:
        """Joint bounds with the pinned joints collapsed to a point.

        SLSQP treats `lb == ub` as a fixed variable, so this is what makes the
        wrist hold its taught angle exactly rather than merely preferring it.
        `pin_joints` uses 1-based joint numbers, matching roboarm's config.
        """
        bounds = list(self.joint_bounds)
        for joint_no, angle_deg in (pin_joints or []):
            index = int(joint_no) - 1
            if not 0 <= index < len(bounds):
                raise ValueError(
                    f"pin joint {joint_no} is out of range 1..{len(bounds)}")
            value = float(np.deg2rad(float(angle_deg)))
            bounds[index] = (value, value)
        return bounds

    def solve_flat(
        self,
        xyz: tuple[float, float, float] | list[float] | np.ndarray,
        rot_matrix: np.ndarray,
        current_joints_rad: np.ndarray | list[float] | None = None,
        teach_joints_deg: list[float] | None = None,
        pin_joints: list[list[float]] | None = DEFAULT_PIN_JOINTS,
        attempts: int | None = None,
        maxiter: int = 120,
        success_cost: float = DEFAULT_FLAT_SUCCESS_COST,
        seed: int = 0,
    ) -> IkResult:
        """Multi-start flat-grasp IK with the wrist joints pinned.

        Mirrors roboarm's `_solve_flat_ik`: the start set is the CURRENT arm
        pose (for continuity — the arm is physically there, so a solution near
        it costs less travel and less collision risk), then the taught flat
        pose, then random perturbations of the taught pose. The seed is fixed
        so a given request always produces the same joint targets; a grasp that
        solved differently on each retry would be very hard to debug.
        """
        target = np.eye(4, dtype=float)
        target[:3, :3] = np.asarray(rot_matrix, dtype=float)
        target[:3, 3] = np.asarray(xyz, dtype=float)

        if current_joints_rad is None:
            current = np.deg2rad(DEFAULT_INIT_DEG)
        else:
            current = np.asarray(current_joints_rad, dtype=float)
        if teach_joints_deg is None:
            teach_joints_deg = DEFAULT_FLAT_TEACH_JOINTS_DEG
        teach = np.deg2rad(np.asarray(teach_joints_deg, dtype=float))

        bounds = self._bounds_with_pins(pin_joints)
        n_starts = int(attempts if attempts is not None
                       else DEFAULT_FLAT_MULTISTART)

        rng = np.random.default_rng(seed)
        starts = [current, teach]
        while len(starts) < max(1, n_starts):
            # Perturb in degrees-of-travel terms, not raw radians: 3 deg of
            # spread around the taught pose is enough to escape a wrong roll
            # branch without wandering into a completely different elbow
            # configuration.
            starts.append(teach + np.deg2rad(rng.uniform(-3.0, 3.0, 6)))

        best_joints = current
        best_cost = float("inf")
        best_pos = float("inf")
        best_rot = float("inf")
        best_msg = "no attempts"
        for guess in starts[: max(1, n_starts)]:
            x0 = np.array(
                [np.clip(v, bounds[i][0], bounds[i][1])
                 for i, v in enumerate(guess)],
                dtype=float,
            )
            result = minimize(
                lambda x: self.flat_cost(x, target)[0],
                x0=x0,
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": int(maxiter), "ftol": 1e-9, "disp": False},
            )
            cost = float(result.fun) if np.isfinite(result.fun) else float("inf")
            if cost < best_cost:
                best_cost = cost
                best_joints = np.asarray(result.x, dtype=float)
                _, best_pos, best_rot = self.flat_cost(best_joints, target)
                best_msg = str(result.message)
            if best_cost <= success_cost:
                break

        return IkResult(
            success=bool(best_cost <= success_cost),
            joints_rad=best_joints,
            cost=best_cost,
            message=best_msg,
            pos_err_m=float(best_pos),
            rot_err_rad=float(best_rot),
        )
