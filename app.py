"""
Virtual Pick-and-Place Robot Simulator.

Run from the project root:

    streamlit run app.py
"""

from __future__ import annotations

import time

import plotly.graph_objects as go
import streamlit as st

from kinematics import (
    CUBE_SIZE,
    DEST_DEG,
    HOME_DEG,
    L1,
    L2,
    L3,
    PICKUP_DEG,
    PLATFORM_HEIGHT,
    PLATFORM_XY,
    ArmPose,
    CycleFrame,
    build_cycle_frames,
    cube_position_for_frame,
    destination_cube_position,
    forward_kinematics,
    pickup_cube_position,
    platform_centers,
)

SPEED_DELAY = {"Slow": 0.11, "Normal": 0.055, "Fast": 0.025}
PAGE_STYLE = """
<style>
.stApp { background: linear-gradient(180deg, #f8fafc 0%, #e8eef7 100%); }
h1, h2, h3 { color: #0f172a; }
.hero {
    background: #0f172a;
    color: #e2e8f0;
    padding: 1rem 1.3rem;
    border-radius: 16px;
    margin-bottom: 0.8rem;
}
.hero h1 { color: #ffffff; margin: 0 0 0.3rem 0; font-size: 1.65rem; }
.hero p { margin: 0; color: #cbd5e1; }
</style>
"""


def _init_state() -> None:
    if "frames" not in st.session_state:
        st.session_state.frames = build_cycle_frames()
    defaults = {
        "frame_index": 0,
        "playing": False,
        "paused": False,
        "cycle_count": 0,
        "speed": "Normal",
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


def _current_frame() -> CycleFrame:
    return st.session_state.frames[st.session_state.frame_index]


def start_cycle() -> None:
    if st.session_state.playing:
        return
    st.session_state.frames = build_cycle_frames()
    st.session_state.frame_index = 0
    st.session_state.playing = True
    st.session_state.paused = False


def pause_cycle() -> None:
    if st.session_state.playing and not st.session_state.paused:
        st.session_state.paused = True


def resume_cycle() -> None:
    if st.session_state.playing and st.session_state.paused:
        st.session_state.paused = False


def reset_scene() -> None:
    st.session_state.frames = build_cycle_frames()
    st.session_state.frame_index = 0
    st.session_state.playing = False
    st.session_state.paused = False
    st.session_state.cycle_count = 0


def robot_status() -> str:
    if st.session_state.paused:
        return "Paused"
    if st.session_state.playing:
        return "Running"
    if _current_frame().operation == "Cycle Complete":
        return "Cycle complete"
    return "Idle"


def _box_mesh(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: str,
    name: str,
    opacity: float = 1.0,
) -> go.Mesh3d:
    cx, cy, cz = center
    dx, dy, dz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    x = [cx - dx, cx + dx, cx + dx, cx - dx, cx - dx, cx + dx, cx + dx, cx - dx]
    y = [cy - dy, cy - dy, cy + dy, cy + dy, cy - dy, cy - dy, cy + dy, cy + dy]
    z = [cz - dz, cz - dz, cz - dz, cz - dz, cz + dz, cz + dz, cz + dz, cz + dz]
    return go.Mesh3d(
        x=x,
        y=y,
        z=z,
        i=[0, 0, 4, 4, 0, 0, 1, 1, 2, 2, 3, 3],
        j=[1, 2, 5, 6, 1, 5, 2, 6, 3, 7, 0, 4],
        k=[2, 3, 6, 7, 5, 4, 6, 5, 7, 6, 4, 7],
        color=color,
        opacity=opacity,
        name=name,
        showscale=False,
        hoverinfo="name",
        flatshading=True,
    )


def _segment(p0, p1, color: str, width: int, name: str) -> go.Scatter3d:
    return go.Scatter3d(
        x=[p0[0], p1[0]],
        y=[p0[1], p1[1]],
        z=[p0[2], p1[2]],
        mode="lines",
        line=dict(color=color, width=width),
        name=name,
        hoverinfo="name",
    )


def _points(points, color: str, size: int, name: str) -> go.Scatter3d:
    return go.Scatter3d(
        x=[p[0] for p in points],
        y=[p[1] for p in points],
        z=[p[2] for p in points],
        mode="markers",
        marker=dict(size=size, color=color),
        name=name,
        hoverinfo="name",
    )


def _gripper_traces(pose: ArmPose, opening: float) -> list:
    gx, gy, gz = pose.gripper
    hx, hy, _ = pose.heading
    ax, ay, az = pose.approach
    side = (-hy, hx, 0.0)
    span = 0.035 + 0.09 * opening
    finger = 0.11
    traces = []
    for sign in (-1.0, 1.0):
        root = (
            gx + side[0] * span * sign - ax * 0.02,
            gy + side[1] * span * sign - ay * 0.02,
            gz + side[2] * span * sign - az * 0.02,
        )
        tip = (
            root[0] + ax * finger,
            root[1] + ay * finger,
            root[2] + az * finger,
        )
        traces.append(_segment(root, tip, "#334155", 10, "Gripper"))
    traces.append(_points([(gx, gy, gz)], "#f43f5e", 8, "Gripper centre"))
    return traces


def build_scene(frame: CycleFrame) -> go.Figure:
    pose = forward_kinematics(*frame.angles_deg)
    cube = cube_position_for_frame(frame)
    pickup_plat, dest_plat = platform_centers()
    pickup_xy = pickup_cube_position()
    dest_xy = destination_cube_position()

    traces = [
        _box_mesh((0.0, 0.0, -0.02), (3.4, 3.4, 0.04), "#cbd5e1", "Floor"),
        _box_mesh((0.0, 0.0, 0.05), (0.36, 0.36, 0.10), "#1e293b", "Base"),
        _box_mesh(pickup_plat, (PLATFORM_XY, PLATFORM_XY, PLATFORM_HEIGHT), "#22c55e", "Pickup platform"),
        _box_mesh(dest_plat, (PLATFORM_XY, PLATFORM_XY, PLATFORM_HEIGHT), "#3b82f6", "Destination platform"),
        _box_mesh(cube, (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), "#ef4444", "Cube"),
        _segment(pose.base, pose.shoulder, "#64748b", 22, "Column L1"),
        _segment(pose.shoulder, pose.elbow, "#2563eb", 18, "Upper arm L2"),
        _segment(pose.elbow, pose.gripper, "#f97316", 16, "Forearm L3"),
        _points([pose.base], "#0f172a", 10, "Base joint"),
        _points([pose.shoulder, pose.elbow], "#eab308", 11, "Arm joints"),
        go.Scatter3d(
            x=[0, 0.55, None, 0, 0, None, 0, 0],
            y=[0, 0, None, 0, 0.55, None, 0, 0],
            z=[0, 0, None, 0, 0, None, 0, 0.55],
            mode="lines",
            line=dict(color="#94a3b8", width=4),
            name="Axes",
            hoverinfo="skip",
        ),
        go.Scatter3d(
            x=[0.58, 0, 0],
            y=[0, 0.58, 0],
            z=[0, 0, 0.58],
            mode="text",
            text=["X", "Y", "Z"],
            textfont=dict(size=12, color="#334155"),
            name="Axis labels",
            hoverinfo="skip",
        ),
        go.Scatter3d(
            x=[pickup_xy[0], dest_xy[0]],
            y=[pickup_xy[1], dest_xy[1]],
            z=[PLATFORM_HEIGHT + 0.28, PLATFORM_HEIGHT + 0.28],
            mode="text",
            text=["PICKUP", "DESTINATION"],
            textfont=dict(size=14, color="#0f172a"),
            name="Station labels",
            hoverinfo="skip",
        ),
    ]
    traces.extend(_gripper_traces(pose, frame.gripper_open))

    fig = go.Figure(data=traces)
    fig.update_layout(
        uirevision="pick-and-place-scene",
        showlegend=False,
        margin=dict(l=0, r=0, t=8, b=0),
        paper_bgcolor="rgba(0,0,0,0)",
        scene=dict(
            xaxis=dict(range=[-0.5, 1.8], title="X (m)", backgroundcolor="#f8fafc", gridcolor="#e2e8f0"),
            yaxis=dict(range=[-1.35, 1.35], title="Y (m)", backgroundcolor="#f8fafc", gridcolor="#e2e8f0"),
            zaxis=dict(range=[-0.05, 1.85], title="Z (m)", backgroundcolor="#eef2ff", gridcolor="#cbd5e1"),
            aspectmode="cube",
            camera=dict(
                eye=dict(x=1.7, y=1.55, z=1.05),
                center=dict(x=0.12, y=0.0, z=0.05),
                up=dict(x=0, y=0, z=1),
            ),
        ),
        height=700,
    )
    return fig


def advance_if_running() -> bool:
    """Advance one frame. Returns True if the caller should rerun."""
    if not st.session_state.playing or st.session_state.paused:
        return False
    last = len(st.session_state.frames) - 1
    if st.session_state.frame_index >= last:
        st.session_state.playing = False
        st.session_state.paused = False
        return False
    time.sleep(SPEED_DELAY.get(st.session_state.speed, 0.055))
    st.session_state.frame_index += 1
    if st.session_state.frame_index >= last:
        st.session_state.frame_index = last
        st.session_state.playing = False
        st.session_state.paused = False
        st.session_state.cycle_count += 1
    return True


def main() -> None:
    st.set_page_config(
        page_title="Virtual Pick-and-Place Robot Simulator",
        page_icon="🦾",
        layout="wide",
        initial_sidebar_state="expanded",
    )
    st.markdown(PAGE_STYLE, unsafe_allow_html=True)
    _init_state()

    st.markdown(
        """
        <div class="hero">
          <h1>Virtual Pick-and-Place Robot Simulator</h1>
          <p>Software-only 3-joint arm that picks a cube from the green platform and places it on the blue platform.</p>
        </div>
        """,
        unsafe_allow_html=True,
    )

    with st.sidebar:
        st.header("Controls")
        running = st.session_state.playing and not st.session_state.paused
        paused = st.session_state.playing and st.session_state.paused

        st.button(
            "Start Pick-and-Place",
            type="primary",
            width="stretch",
            disabled=st.session_state.playing,
            on_click=start_cycle,
        )
        st.button("Pause", width="stretch", disabled=not running, on_click=pause_cycle)
        st.button("Resume", width="stretch", disabled=not paused, on_click=resume_cycle)
        st.button("Reset", width="stretch", on_click=reset_scene)

        st.selectbox("Speed", options=["Slow", "Normal", "Fast"], key="speed")

        st.divider()
        st.caption("Three-joint arm (degrees)")
        frame = _current_frame()
        pose = forward_kinematics(*frame.angles_deg)
        st.write(f"θ1 base: **{pose.joint_angles_deg[0]:.1f}°**")
        st.write(f"θ2 shoulder: **{pose.joint_angles_deg[1]:.1f}°**")
        st.write(f"θ3 elbow: **{pose.joint_angles_deg[2]:.1f}°**")
        st.caption(f"Link lengths: L1={L1:.2f} m, L2={L2:.2f} m, L3={L3:.2f} m")
        grip = pose.gripper
        st.caption(f"Gripper: ({grip[0]:.2f}, {grip[1]:.2f}, {grip[2]:.2f}) m")

    status = robot_status()
    m1, m2, m3 = st.columns(3)
    m1.metric("Current operation", _current_frame().operation)
    m2.metric("Cycle count", st.session_state.cycle_count)
    m3.metric("Robot status", status)

    st.plotly_chart(build_scene(_current_frame()), width="stretch")

    with st.expander("How the pick-and-place process works"):
        st.markdown(
            f"""
            The arm has three rotating joints. Motion is **linear interpolation of joint
            angles** between a few planned poses (home, pickup, destination). There is
            no inverse kinematics and no physics engine.

            1. Start at the home pose {HOME_DEG}.
            2. Move to the pickup pose {PICKUP_DEG} and close the gripper on the cube.
            3. While the gripper is closed, the cube uses the gripper coordinates.
            4. Carry the cube to the destination pose {DEST_DEG}.
            5. Open the gripper; the cube stays on the destination platform.
            6. Return home. **Start** runs another cycle (a new cube appears at pickup).
            **Reset** restores the original scene and sets the cycle count to 0.
            """
        )

    if advance_if_running():
        st.rerun()


main()
