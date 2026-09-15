# Public deploy config for robonix.service.roboarm.arm_ik.
# Values below are the ones this deploy uses; the type/unit/constraint note
# above each key is the contract.
config:
  # ── solver selection ────────────────────────────────────────────────────
  # string, default: vertical; accepted values: vertical, flat.
  # `flat` selects solve_flat(): full-attitude geodesic cost over several
  # starts with the wrist joints pinned — this robot's flat-palm grasp.
  # `vertical` is the upstream position+tilt/yaw decomposition.
  # It must match grasp_pose's grasp_mode: if one is flat and the other
  # vertical the arm reaches a valid but wrongly-oriented pose, which reads as
  # a calibration error.
  grasp_mode: flat

  # string, default: "" — empty means default_urdf_path() walks up from the
  # package and finds urdf/roboarm.urdf (robonix/urdf/roboarm.urdf is a symlink
  # to the description package's generated URDF). Same bytes the
  # roboarm_description primitive serves, which keeps /tf and soma's get_urdf
  # in agreement.
  urdf_path: ""

  # string, default: link6. IK tip frame. The adapter and hand hang off link6,
  # so they are not part of the chain and the solver still sees 6 revolute
  # joints.
  tip_link: link6

  # ── flat solver ─────────────────────────────────────────────────────────
  # list of [joint_number, degrees], 1-BASED joint numbers. These wrist joints
  # are held at their taught angles while the shoulder/elbow absorb the reach.
  pin_joints: [[5, -69.0], [6, 6.75]]

  # float list, degrees, length 6. Taught pose the flat solver starts from and
  # the reference for the pinned joints.
  flat_teach_joints_deg: [-3.867, 90.286, -7.659, 0.764, -74.85, 6.748]

  # float, metres, default: 0.008; must be > 0. Position term weight of the
  # flat cost: cost = (pos_err/weight)^2 + (rot_err/tol)^2.
  flat_ik_pos_weight_m: 0.008

  # float, degrees, default: 5.0; must be > 0. Attitude term weight, above.
  flat_rot_tol_deg: 5.0

  # integer, default: 10; >= 1. Multi-starts for the flat solve.
  flat_ik_multistart: 10

  # float, default: 4.0; must be > 0. Cost ceiling for a flat solve to count as
  # success. With the weights above, 4.0 admits roughly 1.6 cm of combined
  # error.
  flat_ik_success_cost: 4.0

  # integer, default: 120. Iteration cap per start.
  flat_ik_maxiter: 120

  # ── vertical solver (upstream path) ─────────────────────────────────────
  # integer, default: 12; float, default: 9.0; integer, default: 120.
  ik_attempts: 12
  ik_success_cost: 9.0
  ik_maxiter: 120

  # ── lifecycle ───────────────────────────────────────────────────────────
  # list, degrees, length 6. Where the arm goes on init and on reset.
  # ⚠️ BOOTING THIS SERVICE MOVES THE ARM: on_init commands this pose and fails
  # the boot if it cannot get there.
  # The value below came from the reference deploy — taught on this arm WITH a
  # gripper fitted. Re-teach it now that the adapter + hard hand are mounted.
  teach_safe_joints_deg: [3.358211, 2.0, -2.0, -6.708426, 18.580102, 5.898858]

  # list, degrees, length 6; boolean. Pose commanded at startup before
  # teach_safe, and whether to park at all.
  init_joints_deg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  park_on_init: true

  # float, seconds; all must be > 0. Feedback waits and the overall
  # joint-feedback sentinel.
  reset_feedback_timeout_s: 8.0
  init_park_timeout_s: 10.0
  teach_safe_feedback_timeout_s: 8.0
  sentinel_timeout_s: 30.0

  # ── motion shaping ──────────────────────────────────────────────────────
  # integer, default: 50; float, seconds, default: 0.04; float, percent,
  # default: 30.0. A motion is interpolated into `motion_steps` waypoints
  # published every `motion_period_s`.
  motion_steps: 50
  motion_period_s: 0.04
  motion_speed_percent: 30.0

  # float, degrees, default: 3.0; boolean, default: false; float, seconds,
  # default: 2.0. How close every joint must be before a motion counts as
  # reached, whether that check is enforced, and how long to settle.
  reach_threshold_deg: 3.0
  require_reach_feedback: false
  settle_timeout_s: 2.0

  # ── put_down ────────────────────────────────────────────────────────────
  # list, degrees, length 6. The "place the object here" pose. Deliberately NOT
  # set in this deploy: it is specific to a cell's target location, and unset
  # makes put_down report a config error instead of guessing. The flat_grasp
  # skill covers the no-target case by returning the object to where it was
  # picked from.
  # put_down_joints_deg: []
  # float, seconds, default: 10.0 / 8.0 / 0.30.
  put_down_timeout_s: 10.0
  put_down_feedback_timeout_s: 8.0
  put_down_release_pause_s: 0.30

  # ── gripper knobs — INERT ON THIS ROBOT ─────────────────────────────────
  # This arm has no gripper (the arm primitive runs with gripper_exist: false);
  # a LinkerHand O6 is mounted instead and driven by the flat_grasp skill. These
  # keys exist because the service was ported from the gripper deploy; they are
  # accepted, documented for completeness, and do nothing here.
  close_gripper_after_motion: true
  gripper_close_width: 0.0
  gripper_command_repeats: 10
  gripper_effort: 3.0
  gripper_val_mutiple: 2.0
  max_gripper_width: 0.08
  pre_close_pause_s: 0.10
  post_close_pause_s: 0.10
  post_open_pause_s: 0.10
  reset_gripper_width: 0.08
  teach_safe_release_pause_s: 0.30
  reset_post_reach_wait_s: 0.0
