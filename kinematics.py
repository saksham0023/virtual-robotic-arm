"""
Simple 3-joint arm model for the pick-and-place simulator.

The robot is an RRR arm drawn as three segments:
- Segment 1: vertical column that rotates with the base (joint 1)
- Segment 2: upper arm (joint 2 at the shoulder)
- Segment 3: forearm (joint 3 at the elbow)
A gripper is attached at the end of segment 3.

Angles are stored in degrees in the UI and converted to radians here.

    r = L2 * cos(θ2) + L3 * cos(θ2 + θ3)
    z = L1 + L2 * sin(θ2) + L3 * sin(θ2 + θ3)
    x = r * cos(θ1)
    y = r * sin(θ1)

θ1 = 0° points the arm along +X. +Z is up. No inverse kinematics is used;
pick-and-place motion is linear interpolation between planned joint poses.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Segment lengths in metres.
L1 = 0.42  # vertical column
L2 = 0.90  # upper arm
L3 = 0.78  # forearm to gripper centre

CUBE_SIZE = 0.12
PLATFORM_HEIGHT = 0.11
PLATFORM_XY = 0.50
CUBE_Z = PLATFORM_HEIGHT + CUBE_SIZE / 2.0

# Planned joint poses (θ1, θ2, θ3) in degrees. Chosen so forward kinematics
# places the gripper on the pickup / destination platforms.
HOME_DEG = (0.0, 72.0, -40.0)
PICKUP_ABOVE_DEG = (38.0, 42.0, -80.0)
PICKUP_DEG = (38.0, 25.0, -78.0)
DEST_ABOVE_DEG = (-38.0, 42.0, -80.0)
DEST_DEG = (-38.0, 25.0, -78.0)

OPERATIONS = (
    "Idle",
    "Moving to Pickup",
    "Gripping",
    "Moving to Destination",
    "Releasing",
    "Returning Home",
    "Cycle Complete",
)


@dataclass(frozen=True)
class ArmPose:
    """Joint positions and gripper frame from forward kinematics."""

    joint_angles_deg: tuple[float, float, float]
    base: tuple[float, float, float]
    shoulder: tuple[float, float, float]
    elbow: tuple[float, float, float]
    gripper: tuple[float, float, float]
    heading: tuple[float, float, float]
    approach: tuple[float, float, float]


@dataclass(frozen=True)
class CycleFrame:
    """One displayed pose of the pick-and-place cycle."""

    angles_deg: tuple[float, float, float]
    gripper_open: float  # 1.0 = fully open, 0.0 = closed
    cube_attached: bool
    operation: str
    cube_released_at: tuple[float, float, float] | None = None


def _rad(degrees: float) -> float:
    return degrees * math.pi / 180.0


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def forward_kinematics(theta1_deg: float, theta2_deg: float, theta3_deg: float) -> ArmPose:
    """Return joint XYZ positions for the three-segment arm."""
    t1 = _rad(theta1_deg)
    t2 = _rad(theta2_deg)
    t3 = _rad(theta3_deg)

    base = (0.0, 0.0, 0.0)
    shoulder = (0.0, 0.0, L1)

    c1, s1 = math.cos(t1), math.sin(t1)
    c2, s2 = math.cos(t2), math.sin(t2)
    c23, s23 = math.cos(t2 + t3), math.sin(t2 + t3)

    upper = (c1 * c2, s1 * c2, s2)
    forearm = (c1 * c23, s1 * c23, s23)

    elbow = _add(shoulder, _scale(upper, L2))
    gripper = _add(elbow, _scale(forearm, L3))

    return ArmPose(
        joint_angles_deg=(theta1_deg, theta2_deg, theta3_deg),
        base=base,
        shoulder=shoulder,
        elbow=elbow,
        gripper=gripper,
        heading=(c1, s1, 0.0),
        approach=forearm,
    )


def interpolate_angles(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    steps: int,
) -> list[tuple[float, float, float]]:
    """Linear interpolation between two joint poses, including the end pose."""
    if steps < 1:
        raise ValueError("steps must be at least 1")
    poses: list[tuple[float, float, float]] = []
    for i in range(1, steps + 1):
        t = i / steps
        poses.append(
            (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                start[2] + (end[2] - start[2]) * t,
            )
        )
    return poses


def pickup_cube_position() -> tuple[float, float, float]:
    """Cube centre on the pickup platform, from the planned pickup pose."""
    grip = forward_kinematics(*PICKUP_DEG).gripper
    return (grip[0], grip[1], CUBE_Z)


def destination_cube_position() -> tuple[float, float, float]:
    """Cube centre on the destination platform, from the planned drop pose."""
    grip = forward_kinematics(*DEST_DEG).gripper
    return (grip[0], grip[1], CUBE_Z)


def platform_centers() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    pickup = pickup_cube_position()
    dest = destination_cube_position()
    return (
        (pickup[0], pickup[1], PLATFORM_HEIGHT / 2.0),
        (dest[0], dest[1], PLATFORM_HEIGHT / 2.0),
    )


def cube_position_for_frame(frame: CycleFrame) -> tuple[float, float, float]:
    """World position of the cube for a cycle frame."""
    if frame.cube_attached:
        gripper = forward_kinematics(*frame.angles_deg).gripper
        return (gripper[0], gripper[1], gripper[2])
    if frame.cube_released_at is not None:
        return frame.cube_released_at
    return pickup_cube_position()


def build_cycle_frames(move_steps: int = 18, action_steps: int = 8) -> list[CycleFrame]:
    """
    Build one complete pick-and-place cycle as intermediate poses.

    Sequence: home → pickup → grip → destination → release → home.
    """
    frames: list[CycleFrame] = [
        CycleFrame(HOME_DEG, gripper_open=1.0, cube_attached=False, operation="Idle")
    ]

    def extend(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        steps: int,
        operation: str,
        gripper_open: float,
        attached: bool,
        released_at: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]:
        for angles in interpolate_angles(start, end, steps):
            frames.append(
                CycleFrame(
                    angles_deg=angles,
                    gripper_open=gripper_open,
                    cube_attached=attached,
                    operation=operation,
                    cube_released_at=released_at,
                )
            )
        return end

    pose = extend(HOME_DEG, PICKUP_ABOVE_DEG, move_steps, "Moving to Pickup", 1.0, False)
    pose = extend(pose, PICKUP_DEG, max(8, move_steps // 2), "Moving to Pickup", 1.0, False)

    for i in range(1, action_steps + 1):
        opening = 1.0 - i / action_steps
        attached = i == action_steps
        frames.append(
            CycleFrame(
                angles_deg=pose,
                gripper_open=opening,
                cube_attached=attached,
                operation="Gripping",
            )
        )

    pose = extend(pose, PICKUP_ABOVE_DEG, max(8, move_steps // 2), "Moving to Destination", 0.0, True)
    pose = extend(pose, DEST_ABOVE_DEG, move_steps, "Moving to Destination", 0.0, True)
    pose = extend(pose, DEST_DEG, max(8, move_steps // 2), "Moving to Destination", 0.0, True)

    released_at = destination_cube_position()
    for i in range(1, action_steps + 1):
        opening = i / action_steps
        attached = i < action_steps
        frames.append(
            CycleFrame(
                angles_deg=pose,
                gripper_open=opening,
                cube_attached=attached,
                operation="Releasing",
                cube_released_at=None if attached else released_at,
            )
        )

    pose = extend(
        pose, DEST_ABOVE_DEG, max(8, move_steps // 2), "Returning Home", 1.0, False, released_at
    )
    extend(pose, HOME_DEG, move_steps, "Returning Home", 1.0, False, released_at)

    frames.append(
        CycleFrame(
            HOME_DEG,
            gripper_open=1.0,
            cube_attached=False,
            operation="Cycle Complete",
            cube_released_at=released_at,
        )
    )
    return frames
