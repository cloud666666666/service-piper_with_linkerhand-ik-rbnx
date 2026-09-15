#!/usr/bin/env python3
from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any, Optional

import numpy as np
from robonix_api import Err, Ok, Service
from scipy.spatial.transform import Rotation as R

from .solver import (
    DEFAULT_FLAT_MULTISTART,
    DEFAULT_FLAT_SUCCESS_COST,
    DEFAULT_FLAT_TEACH_JOINTS_DEG,
    DEFAULT_INIT_DEG,
    DEFAULT_PIN_JOINTS,
    PiperRoboarmIk,
    parse_float_list,
)


logging.basicConfig(
    level=os.environ.get("ROBOARM_IK_LOG_LEVEL", "INFO"),
    format="[roboarm_ik] %(message)s",
)
log = logging.getLogger("roboarm_ik")

roboarm_ik = Service(
    id=os.environ.get("ROBONIX_CAPABILITY_ID", "roboarm_ik"),
    namespace="robonix/service/manipulation",
)

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JOINT_STATE_NAMES = JOINT_NAMES + ["gripper"]
JOINT_LIMITS_RAD = np.deg2rad(
    [
        [-150.0, 150.0],
        [0.0, 180.0],
        [-170.0, 0.0],
        [-100.0, 100.0],
        [-70.0, 70.0],
        [-120.0, 120.0],
    ]
)

_state_lock = threading.Lock()
_exec_lock = threading.Lock()
_initialized = False
_cfg: dict[str, Any] = {}
_solver: Optional[PiperRoboarmIk] = None

_ros_node = None
_ros_thread: Optional[threading.Thread] = None
_ros_stop_evt = threading.Event()
_joint_cmd_pub = None

_latest_lock = threading.Lock()
_latest_joints_rad: Optional[np.ndarray] = None
_latest_gripper_joint: float = 0.04
_latest_joint_stamp: float = 0.0
_commanded_gripper_width: Optional[float] = None


def _normalize_piper_feedback_joints(joints_rad: np.ndarray) -> np.ndarray:
    """Map wrapped Piper feedback into the SDK joint command ranges."""
    normalized = []
    two_pi = 2.0 * np.pi
    for value, (lower, upper) in zip(np.asarray(joints_rad, dtype=float), JOINT_LIMITS_RAD):
        candidates = [value + k * two_pi for k in range(-2, 3)]
        best = min(
            candidates,
            key=lambda v: 0.0 if lower <= v <= upper else min(abs(v - lower), abs(v - upper)),
        )
        normalized.append(float(np.clip(best, lower, upper)))
    return np.array(normalized, dtype=float)


def _is_flat_mode() -> bool:
    """Whether to route execute_grasp through the flat-palm solver.

    Reads `grasp_mode` from this service's own config, not from grasp_pose's —
    the two are configured independently and both must agree. A mismatch is not
    silently tolerated: if the pose was built for a flat palm but this service
    solves it as a vertical-down gripper pose, the arm goes to a reachable but
    wrong orientation, which looks like a calibration error.
    """
    return str(_cfg.get("grasp_mode", "vertical")).lower() == "flat"


def _current_joints_or_init() -> np.ndarray:
    init = np.deg2rad(parse_float_list(_cfg.get("init_joints_deg"), DEFAULT_INIT_DEG))
    with _latest_lock:
        if _latest_joints_rad is None:
            return init
        return np.array(_latest_joints_rad, dtype=float)


def _wait_for_joint_state(timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with _latest_lock:
            if _latest_joints_rad is not None:
                return True
        time.sleep(0.05)
    return False


def _is_supported_frame(frame_id: str) -> bool:
    return frame_id in {"", "base_link", "arm/base_link"}


def _gripper_width_to_joint(width_m: float) -> float:
    # piper_ctl converts the seventh JointState position from meters to SDK
    # gripper angle and applies its gripper_val_mutiple. Its feedback reports
    # the post-multiplied opening width, so command width / multiplier here.
    multiplier = max(1.0, float(_cfg.get("gripper_val_mutiple", 2.0)))
    max_width = float(_cfg.get("max_gripper_width", 0.08))
    return float(np.clip(float(width_m), 0.0, max_width) / multiplier)


def _current_gripper_width_or(default_width_m: float) -> float:
    with _latest_lock:
        commanded = _commanded_gripper_width
    if commanded is not None and np.isfinite(commanded):
        max_width = float(_cfg.get("max_gripper_width", 0.08))
        return float(np.clip(commanded, 0.0, max_width))

    with _latest_lock:
        width = float(_latest_gripper_joint)
        stamp = float(_latest_joint_stamp)
    if stamp <= 0.0 or not np.isfinite(width):
        return float(default_width_m)
    max_width = float(_cfg.get("max_gripper_width", 0.08))
    return float(np.clip(width, 0.0, max_width))


def _publish_gripper_at_current_pose(width_m: float, count: int = 8) -> None:
    global _commanded_gripper_width
    from sensor_msgs.msg import JointState

    if _ros_node is None or _joint_cmd_pub is None:
        return
    joints = _current_joints_or_init()
    gripper_joint = _gripper_width_to_joint(width_m)
    speed_percent = float(_cfg.get("motion_speed_percent", 30.0))
    gripper_effort = float(_cfg.get("gripper_effort", 3.0))
    period_s = float(_cfg.get("motion_period_s", 0.04))
    for _ in range(max(1, int(count))):
        msg = JointState()
        msg.header.stamp = _ros_node.get_clock().now().to_msg()
        msg.name = list(JOINT_STATE_NAMES)
        msg.position = [float(v) for v in joints] + [gripper_joint]
        msg.velocity = [0.0] * 6 + [speed_percent]
        msg.effort = [0.0] * 6 + [gripper_effort]
        _joint_cmd_pub.publish(msg)
        time.sleep(period_s)
    with _latest_lock:
        _commanded_gripper_width = float(np.clip(
            width_m, 0.0, float(_cfg.get("max_gripper_width", 0.08))
        ))


def _publish_joint_motion(
    target_joints_rad: np.ndarray,
    gripper_width_m: float,
    timeout_s: float,
) -> tuple[bool, str]:
    global _commanded_gripper_width
    from sensor_msgs.msg import JointState

    if _ros_node is None or _joint_cmd_pub is None:
        return False, "ROS bridge not ready"

    t0 = time.monotonic()
    start = _current_joints_or_init()
    target = np.asarray(target_joints_rad, dtype=float)
    steps = int(_cfg.get("motion_steps", 50))
    period_s = float(_cfg.get("motion_period_s", 0.04))
    speed_percent = float(_cfg.get("motion_speed_percent", 30.0))
    gripper_effort = float(_cfg.get("gripper_effort", 3.0))
    target_gripper_width = float(gripper_width_m)
    current_gripper_width = _current_gripper_width_or(target_gripper_width)
    close_after_motion = (
        bool(_cfg.get("close_gripper_after_motion", True))
        and target_gripper_width < current_gripper_width - 0.002
    )
    open_before_motion = target_gripper_width > current_gripper_width + 0.002
    motion_gripper_width = (
        current_gripper_width if close_after_motion else target_gripper_width
    )
    gripper_joint = _gripper_width_to_joint(motion_gripper_width)

    if open_before_motion:
        _publish_gripper_at_current_pose(
            target_gripper_width,
            count=int(_cfg.get("gripper_command_repeats", 10)),
        )
        time.sleep(float(_cfg.get("post_open_pause_s", 0.10)))
        motion_gripper_width = target_gripper_width
        gripper_joint = _gripper_width_to_joint(motion_gripper_width)

    for alpha in np.linspace(0.0, 1.0, max(2, steps + 1))[1:]:
        joints = start * (1.0 - alpha) + target * alpha
        msg = JointState()
        msg.header.stamp = _ros_node.get_clock().now().to_msg()
        msg.name = list(JOINT_STATE_NAMES)
        msg.position = [float(v) for v in joints] + [gripper_joint]
        msg.velocity = [0.0] * 6 + [speed_percent]
        msg.effort = [0.0] * 6 + [gripper_effort]
        _joint_cmd_pub.publish(msg)
        time.sleep(period_s)

    if not close_after_motion:
        with _latest_lock:
            _commanded_gripper_width = float(np.clip(
                target_gripper_width,
                0.0,
                float(_cfg.get("max_gripper_width", 0.08)),
            ))

    if close_after_motion:
        remaining_s = max(0.1, timeout_s - (time.monotonic() - t0))
        reached, reach_msg = _wait_for_joint_target(target, timeout_s=remaining_s)
        if not reached:
            return False, f"target not reached before gripper close; {reach_msg}"
        time.sleep(float(_cfg.get("pre_close_pause_s", 0.10)))
        _publish_gripper_at_current_pose(
            target_gripper_width,
            count=int(_cfg.get("gripper_command_repeats", 10)),
        )
        time.sleep(float(_cfg.get("post_close_pause_s", 0.10)))
        return True, "ok; gripper closed after target reached"

    settle_s = min(float(_cfg.get("settle_timeout_s", 2.0)), max(0.0, timeout_s))
    threshold_rad = np.deg2rad(float(_cfg.get("reach_threshold_deg", 3.0)))
    require_feedback = bool(_cfg.get("require_reach_feedback", False))
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        with _latest_lock:
            current = None if _latest_joints_rad is None else np.array(_latest_joints_rad)
        if current is not None and float(np.max(np.abs(current - target))) <= threshold_rad:
            return True, "ok"
        time.sleep(0.05)

    msg = "command_published; feedback did not settle before timeout"
    return (False, msg) if require_feedback else (True, msg)


def _execute_target(
    frame_id: str,
    xyz: tuple[float, float, float],
    quat_xyzw: tuple[float, float, float, float],
    gripper_width: float,
    timeout_s: float,
) -> tuple[bool, str, float]:
    t0 = time.monotonic()
    if not _exec_lock.acquire(blocking=False):
        return False, "executor busy", 0.0
    try:
        with _state_lock:
            solver = _solver
            initialized = _initialized
        if not initialized or solver is None:
            return False, "roboarm_ik not initialized", time.monotonic() - t0
        if not _is_supported_frame(frame_id):
            return (
                False,
                f"unsupported target frame {frame_id!r}; expected arm/base_link",
                time.monotonic() - t0,
            )

        current = _current_joints_or_init()
        if _is_flat_mode():
            # Flat-palm grasp: position + FULL-attitude target, wrist joints
            # pinned. Uses solve_flat rather than solve_xyz_quat because the
            # latter's attitude cost splits into tool-z tilt + world-z yaw,
            # which stops tracking roll once the palm is parallel to the table
            # — and roll is what decides whether the palm actually lands flat.
            ik = solver.solve_flat(
                xyz,
                R.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix(),
                current_joints_rad=current,
                teach_joints_deg=parse_float_list(
                    _cfg.get("flat_teach_joints_deg"),
                    DEFAULT_FLAT_TEACH_JOINTS_DEG,
                ),
                pin_joints=_cfg.get("pin_joints", DEFAULT_PIN_JOINTS),
                attempts=int(_cfg.get("flat_ik_multistart",
                                      DEFAULT_FLAT_MULTISTART)),
                maxiter=int(_cfg.get("flat_ik_maxiter", 120)),
                success_cost=float(_cfg.get(
                    "flat_ik_success_cost", DEFAULT_FLAT_SUCCESS_COST)),
            )
            log.info(
                "flat IK: success=%s cost=%.4f pos_err=%.2f mm rot_err=%.2f deg "
                "joints_deg=%s msg=%s",
                ik.success, ik.cost, ik.pos_err_m * 1000.0, ik.rot_err_deg,
                [round(v, 2) for v in ik.joints_deg], ik.message,
            )
        else:
            ik = solver.solve_xyz_quat(
                xyz,
                quat_xyzw,
                current_joints_rad=current,
                init_joints_rad=np.deg2rad(
                    parse_float_list(_cfg.get("init_joints_deg"), DEFAULT_INIT_DEG)
                ),
                attempts=int(_cfg.get("ik_attempts", 12)),
                maxiter=int(_cfg.get("ik_maxiter", 120)),
                success_cost=float(_cfg.get("ik_success_cost", 9.0)),
            )
            log.info(
                "IK result success=%s cost=%.4f joints_deg=%s msg=%s",
                ik.success,
                ik.cost,
                [round(v, 2) for v in ik.joints_deg],
                ik.message,
            )
        if not ik.success:
            return False, f"ik_failed cost={ik.cost:.4f}: {ik.message}", time.monotonic() - t0

        remaining = max(0.1, timeout_s - (time.monotonic() - t0))
        motion_ok, motion_msg = _publish_joint_motion(
            ik.joints_rad,
            gripper_width_m=gripper_width,
            timeout_s=remaining,
        )
        return motion_ok, motion_msg, time.monotonic() - t0
    except Exception as e:  # noqa: BLE001
        return False, str(e), time.monotonic() - t0
    finally:
        _exec_lock.release()


def _reset_motion(timeout_s: float = 10.0) -> tuple[bool, str, float]:
    t0 = time.monotonic()
    if not _exec_lock.acquire(blocking=False):
        return False, "executor busy", 0.0
    try:
        init_joints = np.deg2rad(parse_float_list(_cfg.get("init_joints_deg"), DEFAULT_INIT_DEG))
        gripper_width = float(_cfg.get("reset_gripper_width", 0.08))
        log.info(
            "reset target joints_deg=%s gripper=%.3f",
            [round(v, 2) for v in np.rad2deg(init_joints).astype(float).tolist()],
            gripper_width,
        )
        ok, msg = _publish_joint_motion(init_joints, gripper_width, timeout_s=timeout_s)
        if ok:
            ok, msg = _wait_for_joint_target(
                init_joints,
                timeout_s=float(_cfg.get("reset_feedback_timeout_s", 8.0)),
            )
        if ok:
            time.sleep(max(0.0, float(_cfg.get("reset_post_reach_wait_s", 0.0))))
            return True, "reset complete; target reached", time.monotonic() - t0
        return False, f"reset failed: {msg}", time.monotonic() - t0
    finally:
        _exec_lock.release()


def _wait_for_joint_target(target_joints_rad: np.ndarray, timeout_s: float) -> tuple[bool, str]:
    threshold_rad = np.deg2rad(float(_cfg.get("reach_threshold_deg", 3.0)))
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_err = None
    while time.monotonic() < deadline:
        with _latest_lock:
            current = None if _latest_joints_rad is None else np.array(_latest_joints_rad)
        if current is not None:
            last_err = float(np.max(np.abs(current - target_joints_rad)))
            if last_err <= threshold_rad:
                return True, "target reached"
        time.sleep(0.05)
    if last_err is None:
        return False, "no joint feedback while waiting for target"
    return False, f"target not reached; max joint error {np.rad2deg(last_err):.2f} deg"


def _parse_waypoint_list_deg(cfg_key: str) -> list[np.ndarray]:
    raw = _cfg.get(cfg_key, []) or []
    if not isinstance(raw, list):
        raise ValueError(f"{cfg_key} must be a list of 6-joint degree lists")
    waypoints: list[np.ndarray] = []
    for idx, item in enumerate(raw):
        try:
            values = parse_float_list(item, [])
        except Exception as e:  # noqa: BLE001
            raise ValueError(f"{cfg_key}[{idx}] is invalid: {e}") from e
        if len(values) != len(JOINT_NAMES):
            raise ValueError(
                f"{cfg_key}[{idx}] must have {len(JOINT_NAMES)} joints, got {len(values)}"
            )
        waypoints.append(np.deg2rad(values))
    return waypoints


def _publish_joint_motion_and_wait(
    target_joints_rad: np.ndarray,
    gripper_width: float,
    timeout_s: float,
    feedback_timeout_s: float,
) -> tuple[bool, str]:
    ok, msg = _publish_joint_motion(
        target_joints_rad,
        gripper_width,
        timeout_s=timeout_s,
    )
    if ok:
        ok, msg = _wait_for_joint_target(
            target_joints_rad,
            timeout_s=feedback_timeout_s,
        )
    return ok, msg


def _teach_safe_motion(
    timeout_s: float = 10.0,
    hold_gripper: bool = False,
) -> tuple[bool, str, float]:
    t0 = time.monotonic()
    if not _exec_lock.acquire(blocking=False):
        return False, "executor busy", 0.0
    try:
        joints = np.deg2rad(
            parse_float_list(_cfg.get("teach_safe_joints_deg"), DEFAULT_INIT_DEG)
        )
        release_gripper_width = float(_cfg.get("max_gripper_width", 0.08))
        gripper_width = (
            _current_gripper_width_or(float(_cfg.get("gripper_close_width", 0.0)))
            if hold_gripper else release_gripper_width
        )
        log.info(
            "teach-safe target joints_deg=%s gripper=%.3f hold_gripper=%s",
            [round(v, 2) for v in np.rad2deg(joints).astype(float).tolist()],
            gripper_width,
            bool(hold_gripper),
        )
        if hold_gripper:
            try:
                waypoints = _parse_waypoint_list_deg("carry_to_teach_safe_waypoints_deg")
            except ValueError as e:
                return False, str(e), time.monotonic() - t0
            if waypoints:
                log.info(
                    "teach-safe hold: moving through %d carry waypoint(s)",
                    len(waypoints),
                )
        else:
            waypoints = []
            log.info("teach-safe release: opening gripper at current pose before moving")
            _publish_gripper_at_current_pose(
                gripper_width,
                count=int(_cfg.get("gripper_command_repeats", 10)),
            )
            time.sleep(float(_cfg.get("teach_safe_release_pause_s", 0.30)))

        feedback_timeout_s = float(_cfg.get("teach_safe_feedback_timeout_s", 8.0))
        segment_timeout_s = max(
            0.1,
            timeout_s / max(1, len(waypoints) + 1),
        )
        for idx, waypoint in enumerate(waypoints, start=1):
            log.info(
                "teach-safe hold waypoint %d/%d joints_deg=%s",
                idx,
                len(waypoints),
                [round(v, 2) for v in np.rad2deg(waypoint).astype(float).tolist()],
            )
            ok, msg = _publish_joint_motion_and_wait(
                waypoint,
                gripper_width,
                timeout_s=segment_timeout_s,
                feedback_timeout_s=feedback_timeout_s,
            )
            if not ok:
                return (
                    False,
                    f"teach-safe carry waypoint {idx} failed: {msg}",
                    time.monotonic() - t0,
                )

        ok, msg = _publish_joint_motion_and_wait(
            joints,
            gripper_width,
            timeout_s=segment_timeout_s,
            feedback_timeout_s=feedback_timeout_s,
        )
        if ok:
            if hold_gripper:
                return True, "teach-safe complete; gripper held", time.monotonic() - t0
            return True, "teach-safe complete; arm ready to disable", time.monotonic() - t0
        return False, f"teach-safe failed: {msg}", time.monotonic() - t0
    finally:
        _exec_lock.release()


def _put_down_motion(timeout_s: float = 10.0) -> tuple[bool, str, float]:
    """Move to the configured release pose, open, then return teach-safe."""
    t0 = time.monotonic()
    if not _exec_lock.acquire(blocking=False):
        return False, "executor busy", 0.0
    try:
        raw_put_down_joints = _cfg.get("put_down_joints_deg")
        if raw_put_down_joints is None:
            return False, "put_down_joints_deg is not configured", time.monotonic() - t0
        try:
            put_down_values = parse_float_list(raw_put_down_joints, [])
            teach_safe_values = parse_float_list(
                _cfg.get("teach_safe_joints_deg"), DEFAULT_INIT_DEG
            )
        except Exception as e:  # noqa: BLE001
            return False, f"invalid put-down configuration: {e}", time.monotonic() - t0
        for key, values in (
            ("put_down_joints_deg", put_down_values),
            ("teach_safe_joints_deg", teach_safe_values),
        ):
            if len(values) != len(JOINT_NAMES):
                return (
                    False,
                    f"{key} must have {len(JOINT_NAMES)} joints, got {len(values)}",
                    time.monotonic() - t0,
                )

        put_down_joints = np.deg2rad(put_down_values)
        teach_safe_joints = np.deg2rad(teach_safe_values)
        held_gripper_width = _current_gripper_width_or(
            float(_cfg.get("gripper_close_width", 0.0))
        )
        release_gripper_width = float(_cfg.get("max_gripper_width", 0.08))
        feedback_timeout_s = float(_cfg.get("put_down_feedback_timeout_s", 8.0))
        segment_timeout_s = max(0.1, timeout_s / 2.0)

        log.info(
            "put-down: carrying object to joints_deg=%s gripper=%.3f",
            [round(v, 2) for v in put_down_values],
            held_gripper_width,
        )
        ok, msg = _publish_joint_motion_and_wait(
            put_down_joints,
            held_gripper_width,
            timeout_s=segment_timeout_s,
            feedback_timeout_s=feedback_timeout_s,
        )
        if not ok:
            return (
                False,
                f"put-down pose failed; gripper remains held: {msg}",
                time.monotonic() - t0,
            )

        log.info("put-down: opening gripper at fixed release pose")
        _publish_gripper_at_current_pose(
            release_gripper_width,
            count=int(_cfg.get("gripper_command_repeats", 10)),
        )
        time.sleep(float(_cfg.get("put_down_release_pause_s", 0.30)))

        log.info(
            "put-down: returning to teach-safe joints_deg=%s with gripper open",
            [round(v, 2) for v in teach_safe_values],
        )
        ok, msg = _publish_joint_motion_and_wait(
            teach_safe_joints,
            release_gripper_width,
            timeout_s=segment_timeout_s,
            feedback_timeout_s=feedback_timeout_s,
        )
        if not ok:
            return (
                False,
                f"object released, but teach-safe return failed: {msg}",
                time.monotonic() - t0,
            )
        return (
            True,
            "put-down complete; gripper released and arm returned to teach-safe",
            time.monotonic() - t0,
        )
    finally:
        _exec_lock.release()


def _ros_thread_main() -> None:
    global _ros_node, _joint_cmd_pub
    global _latest_joints_rad, _latest_gripper_joint, _latest_joint_stamp

    import rclpy
    from piper_msgs.msg import PiperStatusMsg
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    rclpy.init(args=None)
    node = Node("roboarm_ik_executor")
    callback_group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    _ros_node = node
    _joint_cmd_pub = node.create_publisher(JointState, "/arm/joint_command", 10)

    def _joint_cb(msg: JointState) -> None:
        positions = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        joints = [positions.get(name) for name in JOINT_NAMES]
        if any(v is None for v in joints):
            return
        with _latest_lock:
            globals()["_latest_joints_rad"] = _normalize_piper_feedback_joints(
                np.array(joints, dtype=float)
            )
            globals()["_latest_gripper_joint"] = float(positions.get("gripper", 0.04))
            globals()["_latest_joint_stamp"] = time.monotonic()

    def _reset_srv_cb(_req: Trigger.Request, _resp: Trigger.Response) -> Trigger.Response:
        resp = Trigger.Response()
        log.warning("ROS service requested: /moveit_control/reset")
        ok, reason, _elapsed = _reset_motion(timeout_s=10.0)
        log.warning(
            "ROS service completed: /moveit_control/reset success=%s message=%r",
            bool(ok),
            reason,
        )
        resp.success = bool(ok)
        resp.message = str(reason)
        return resp

    def _teach_safe_srv_cb(_req: Trigger.Request, _resp: Trigger.Response) -> Trigger.Response:
        resp = Trigger.Response()
        log.warning("ROS service requested: /moveit_control/teach_safe")
        ok, reason, _elapsed = _teach_safe_motion(timeout_s=10.0)
        log.warning(
            "ROS service completed: /moveit_control/teach_safe success=%s message=%r",
            bool(ok),
            reason,
        )
        resp.success = bool(ok)
        resp.message = str(reason)
        return resp

    # arm_status is subscribed as a liveness hint for parity with piper_moveit.
    def _status_cb(_msg: PiperStatusMsg) -> None:
        return

    node.create_subscription(
        JointState,
        "/arm/joint_states_single",
        _joint_cb,
        10,
        callback_group=callback_group,
    )
    node.create_subscription(
        PiperStatusMsg,
        "/arm/arm_status",
        _status_cb,
        10,
        callback_group=callback_group,
    )
    node.create_service(
        Trigger,
        "/moveit_control/reset",
        _reset_srv_cb,
        callback_group=callback_group,
    )
    node.create_service(
        Trigger,
        "/moveit_control/teach_safe",
        _teach_safe_srv_cb,
        callback_group=callback_group,
    )

    log.info(
        "ROS bridge up: publish /arm/joint_command, services /moveit_control/reset + /moveit_control/teach_safe"
    )
    while not _ros_stop_evt.is_set():
        executor.spin_once(timeout_sec=0.1)
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    log.info("ROS bridge exited")


@roboarm_ik.on_init
def init(cfg):
    global _initialized, _cfg, _solver, _ros_thread
    with _state_lock:
        if _initialized:
            return Ok()

    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as e:
            return Err(f"bad config_json: {e}")
    _cfg = dict(cfg)

    try:
        _solver = PiperRoboarmIk(
            urdf_path=_cfg.get("urdf_path") or None,
            tip_link=str(_cfg.get("tip_link", "link6")),
        )
    except Exception as e:  # noqa: BLE001
        return Err(f"IK solver init failed: {e}")

    _ros_stop_evt.clear()
    _ros_thread = threading.Thread(
        target=_ros_thread_main, name="roboarm_ik-ros", daemon=True
    )
    _ros_thread.start()
    time.sleep(0.5)

    sentinel_timeout = float(_cfg.get("sentinel_timeout_s", 30.0))
    if not _wait_for_joint_state(sentinel_timeout):
        _ros_stop_evt.set()
        return Err(
            f"sentinel: no /arm/joint_states_single within {sentinel_timeout:.1f}s "
            "(is piper_ctl ACTIVE?)"
        )

    ok, reason, elapsed = _teach_safe_motion(
        timeout_s=float(_cfg.get("init_park_timeout_s", 10.0))
    )
    if not ok:
        _ros_stop_evt.set()
        return Err(f"init teach-safe failed: {reason}")
    log.info("init teach-safe complete: msg=%r elapsed=%.2fs", reason, elapsed)
    # if bool(_cfg.get("park_on_init", True)):
    #     ok, reason, elapsed = _reset_motion(
    #         timeout_s=float(_cfg.get("init_park_timeout_s", 10.0))
    #     )
    #     if not ok:
    #         _ros_stop_evt.set()
    #         return Err(f"init park failed: {reason}")
    #     log.info("init park complete: msg=%r elapsed=%.2fs", reason, elapsed)

    with _state_lock:
        _initialized = True
    log.info("init complete: roboarm IK manipulation executor active")
    return Ok()


@roboarm_ik.on_deactivate
def deactivate():
    global _initialized
    _ros_stop_evt.set()
    if _ros_thread is not None:
        _ros_thread.join(timeout=5.0)
    with _state_lock:
        _initialized = False
    return Ok()


import manipulation_pb2  # noqa: E402  pylint: disable=wrong-import-position


@roboarm_ik.grpc("robonix/service/manipulation/execute_grasp")
def execute_grasp(req: manipulation_pb2.ExecuteGrasp_Request) -> manipulation_pb2.ExecuteGrasp_Response:
    ps = req.target_pose
    ok, reason, elapsed = _execute_target(
        ps.header.frame_id,
        (
            float(ps.pose.position.x),
            float(ps.pose.position.y),
            float(ps.pose.position.z),
        ),
        (
            float(ps.pose.orientation.x),
            float(ps.pose.orientation.y),
            float(ps.pose.orientation.z),
            float(ps.pose.orientation.w),
        ),
        float(req.gripper_width),
        float(req.timeout_s) if req.timeout_s > 0 else 20.0,
    )
    return manipulation_pb2.ExecuteGrasp_Response(
        success=bool(ok), message=str(reason), elapsed_s=float(elapsed))


@roboarm_ik.grpc("robonix/service/manipulation/reset")
def reset(_req: manipulation_pb2.Reset_Request) -> manipulation_pb2.Reset_Response:
    log.warning("gRPC service requested: robonix/service/manipulation/reset")
    ok, reason, elapsed = _reset_motion(timeout_s=10.0)
    log.warning(
        "gRPC service completed: robonix/service/manipulation/reset success=%s message=%r",
        bool(ok),
        reason,
    )
    return manipulation_pb2.Reset_Response(
        success=bool(ok), message=str(reason), elapsed_s=float(elapsed))


@roboarm_ik.grpc("robonix/service/manipulation/teach_safe")
def teach_safe(_req: manipulation_pb2.TeachSafe_Request) -> manipulation_pb2.TeachSafe_Response:
    log.warning("gRPC service requested: robonix/service/manipulation/teach_safe")
    ok, reason, elapsed = _teach_safe_motion(
        timeout_s=10.0,
        hold_gripper=bool(getattr(_req, "hold_gripper", False)),
    )
    log.warning(
        "gRPC service completed: robonix/service/manipulation/teach_safe success=%s message=%r",
        bool(ok),
        reason,
    )
    return manipulation_pb2.TeachSafe_Response(
        success=bool(ok), message=str(reason), elapsed_s=float(elapsed))


@roboarm_ik.grpc("robonix/service/manipulation/put_down")
def put_down(_req: manipulation_pb2.PutDown_Request) -> manipulation_pb2.PutDown_Response:
    log.warning("gRPC service requested: robonix/service/manipulation/put_down")
    ok, reason, elapsed = _put_down_motion(
        timeout_s=float(_cfg.get("put_down_timeout_s", 10.0))
    )
    log.warning(
        "gRPC service completed: robonix/service/manipulation/put_down success=%s message=%r",
        bool(ok),
        reason,
    )
    return manipulation_pb2.PutDown_Response(
        success=bool(ok), message=str(reason), elapsed_s=float(elapsed))


if __name__ == "__main__":
    roboarm_ik.run()
