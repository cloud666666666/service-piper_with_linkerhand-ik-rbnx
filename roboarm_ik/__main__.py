from __future__ import annotations

import argparse
import sys

import numpy as np

from .solver import DEFAULT_INIT_DEG, PiperRoboarmIk


def main() -> int:
    parser = argparse.ArgumentParser(description="Solve a Piper vertical-down IK target.")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw in radians.")
    parser.add_argument("--urdf-path", default=None)
    parser.add_argument("--attempts", type=int, default=12)
    parser.add_argument("--maxiter", type=int, default=120)
    args = parser.parse_args()

    solver = PiperRoboarmIk(urdf_path=args.urdf_path)
    result = solver.solve_vertical_down(
        [args.x, args.y, args.z],
        yaw_rad=args.yaw,
        current_joints_rad=np.deg2rad(DEFAULT_INIT_DEG),
        attempts=args.attempts,
        maxiter=args.maxiter,
    )
    print(f"success={result.success} cost={result.cost:.6f} message={result.message}")
    print("joints_deg=", [round(value, 4) for value in result.joints_deg])
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
