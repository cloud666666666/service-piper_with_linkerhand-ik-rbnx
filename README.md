# roboarm_ik_rbnx

Robonix-managed manipulation executor for the Piper arm.

> **Provenance — vendored repackaging.** Copied from
> `syswonder/service-roboarm-ik-rbnx` @ `919fdfc` (main, 2026-09-15) so the
> roboarm deploy owns everything but the camera package. License unchanged
> (MulanPSL-2.0). **Local difference:** `grasp_mode: flat` → `solve_flat()`
> (full-attitude geodesic cost, wrist joints pinned) in addition to upstream's
> vertical solver. Everything below describes the shared, upstream behaviour
> unless it says `flat`.

It is an alternative to `piper_moveit_rbnx`: start exactly one of them in
`robonix_manifest.yaml`. Both expose `robonix/service/manipulation/*`; running
both at the same time can make atlas resolution ambiguous and can double-command
the arm.

Execution path:

```text
pick_skill/yolo_grasp -> manipulation/execute_grasp
  -> roboarm IK on Piper URDF link6
  -> /arm/joint_command
  -> piper_ctl_rbnx
  -> CAN
```

ROS compatibility:

- subscribes `/arm/joint_states_single` (proprio feedback from `primitive/arm/joint_states`)
- publishes `/arm/joint_command` (joint target for `primitive/arm/joint_command`)
- exposes `/moveit_control/reset` as `std_srvs/srv/Trigger`

It does not start MoveIt, `move_group`, or the C++ moveit control node.

For `pick_skill.put_down`, configure `put_down_joints_deg` with six joint
angles in degrees. The executor carries the held object to that pose, opens the
gripper there, and then returns the arm to `teach_safe_joints_deg`. If the
put-down pose cannot be reached, it reports failure without opening the gripper.
