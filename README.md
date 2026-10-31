# Virtual Pick-and-Place Robot Simulator

A software-only college mini project. A three-joint robot arm picks a cube from a **PICKUP** platform and places it on a **DESTINATION** platform. There is no physical hardware.

## Features

- 3D workstation: floor, base, three arm segments, joints, gripper, two platforms, and one cube
- Automatic pick-and-place cycle with intermediate poses (not an instant jump)
- Working **Start**, **Pause**, **Resume**, **Reset**, and speed control
- Live operation name, robot status, cycle count, and joint angles
- Cube travels with the gripper, then stays on the destination after release

## Technologies

- Python 3.10+
- Streamlit
- Plotly
- Pytest (for the arm model and cycle)

## Installation

```bash
cd /path/to/virtual-robotic-arm
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

On Windows PowerShell use `.\.venv\Scripts\Activate.ps1` instead of `source .venv/bin/activate`.

## Run

```bash
streamlit run app.py
```

Open the URL Streamlit prints, usually `http://localhost:8501`.

## Controls

| Control | Action |
| --- | --- |
| Start Pick-and-Place | Runs one cycle from the home pose |
| Pause | Freezes the current pose |
| Resume | Continues a paused cycle |
| Reset | Restores home pose, cube on pickup, cycle count 0 |
| Speed | Slow, Normal, or Fast |

Start is disabled while a cycle is running so two animations cannot overlap. After a cycle finishes, Start runs again with a new cube on the pickup platform.

## Tests

```bash
pytest -q
```

## Limitations

- No motors, sensors, gravity, or collision physics
- No inverse kinematics; motion is interpolation between planned joint poses
- The 3D view uses simple geometry, not photorealistic rendering
