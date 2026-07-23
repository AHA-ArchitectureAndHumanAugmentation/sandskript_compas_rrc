# COMPAS RRC workflow for Sandskript

This repository contains the COMPAS RRC workflow developed for the Sandskript project, part of Sybil at Ars Electronica 2026. It sends toolpaths stored as JSON files to an ABB GoFa robot through a Docker-based ROS communication setup.

The repository contains:

- A Grasshopper definition for generating COMPAS toolpath frames
- Example line and circle toolpaths stored as COMPAS JSON data
- A Python script for testing and executing the toolpath on the robot
- Digital output control for the spraying valve
- A Conda environment for the Python dependencies
- A Docker Compose configuration for ROS, rosbridge, and the ABB driver

> **Safety:** This repository controls a physical industrial robot and a pneumatic spraying output. Test each movement stage separately, use reduced robot speed, keep the emergency stop accessible, and verify the tool, work object, offsets, and workspace before executing the complete toolpath.

## Repository Structure

```text
sandskript_compas_rrc/
├── 301_toolpath_generator.ghx   # Grasshopper toolpath generator
├── 302_send_to_robot.py         # ABB movement and spray-control script
├── env_compas_rrc.yml           # Conda environment definition
├── data/
│   ├── toolpath.json            # Example wavy toolpath, 15 frames
│   ├── toolpath_circle.json     # Example circular toolpath, 15 frames
│   └── toolpath_line.json       # Example linear toolpath, 15 frames
└── docker/
    └── docker-compose.yml       # ROS and ABB driver services
```

## Workflow

```text
Rhino / Grasshopper
        ↓
Generate ordered COMPAS Frames
        ↓
Export frames to JSON
        ↓
Load JSON in Python
        ↓
Apply toolpath translation
        ↓
Connect to ROS and ABB
        ↓
Capture the nozzle orientation at HOME_CONFIG
        ↓
Approach the toolpath safely
        ↓
Turn the spray output ON
        ↓
Follow the blended linear toolpath
        ↓
Turn the spray output OFF
        ↓
Retract and return home
```

## 1. Requirements

### Software

- Windows
- Conda or Miniconda
- Docker Desktop
- Rhino 8
- Grasshopper
- ABB RobotWare system configured for `compas_rrc`

### Hardware

- ABB GoFa robot
- ABB controller and FlexPendant
- Spraying tool configured on the robot
- Pneumatic valve connected to a robot digital output
- Computer connected to the ABB controller network

## 2. Install the Conda Environment

Open Anaconda Prompt or a terminal in the repository folder.

Create the environment:

```bash
conda env create -f env_compas_rrc.yml
```

Activate it:

```bash
conda activate compas_rrc
```

Verify the main imports:

```bash
python -c "import compas; import compas_rrc; import compas_fab; import roslibpy; print('Environment works')"
```

To update an existing environment after modifying the YAML file:

```bash
conda env update -f env_compas_rrc.yml --prune
```

The environment currently includes:

| Package | Version | Purpose |
|---|---:|---|
| Python | 3.12.12 | Python runtime |
| COMPAS | 2.14.1 | Geometry and data framework |
| compas_rrc | 2.0.0 | Communication with ABB through ROS |
| compas_fab | 1.1.0 | Robotic fabrication tools |
| compas_robots | 0.6.0 | Robot-model tools |
| roslibpy | 1.8.1 | Python connection to rosbridge |
| Autobahn | 24.4.2 | WebSocket communication |

The supplied environment contains Windows-specific dependencies and is intended primarily for Windows computers.

## 3. Start the ROS and ABB Services

The Docker configuration starts three services:

| Service | Purpose | Port |
|---|---|---:|
| `ros-master` | Runs `roscore` | `11311` |
| `ros-bridge` | Runs the rosbridge WebSocket server | `9090` |
| `abb-driver` | Connects ROS to the ABB controller | ABB streaming/state ports |

From the repository folder, start the services with:

```bash
cd docker
docker compose up
```

To run them in the background:

```bash
docker compose up -d
```

To stop them:

```bash
docker compose down
```

The current ABB driver configuration uses:

```yaml
robot_ip: 192.168.125.1
robot_streaming_port: 30101
robot_state_port: 30201
namespace: rob1
```

`192.168.125.1` is configured as the real controller service-port address. Change `robot_ip` in `docker/docker-compose.yml` when using another controller address or a virtual controller.

## 4. Generate a Toolpath

Open:

```text
301_toolpath_generator.ghx
```

The Grasshopper definition should generate an ordered list of COMPAS `Frame` objects and export them to a JSON file in `data/`.

The repository includes three examples:

| File | Frames | Geometry |
|---|---:|---|
| `data/toolpath.json` | 15 | Line from approximately X = 150 mm to X = 850 mm |
| `data/toolpath_line.json` | 15 | Line from X = 200 mm to X = 800 mm |
| `data/toolpath_circle.json` | 15 | Closed circular path |

All three example paths are initially located around Y = 500 mm and Z = 0 mm before the Python offset is applied.

## 5. Toolpath Data Format

Each JSON file contains a `frames` list:

```json
{
  "frames": [
    {
      "dtype": "compas.geometry/Frame",
      "data": {
        "point": [200.0, 500.0, 0.0],
        "xaxis": [1.0, 0.0, 0.0],
        "yaxis": [0.0, -1.0, 0.0]
      }
    }
  ]
}
```

| Attribute | Type | Description |
|---|---|---|
| `point` | `[x, y, z]` | TCP position in millimetres |
| `xaxis` | `[x, y, z]` | Frame X-axis |
| `yaxis` | `[x, y, z]` | Frame Y-axis |
| `frames` | list | Ordered robot targets |

The Python script validates that the JSON contains a non-empty `frames` list.

## 6. Configure the Robot Script

Open:

```text
302_send_to_robot.py
```

### Robot definitions

```python
HOME_CONFIG = [90.0, 15.0, -150.0, -5.0, -40.0, -215.0]
TOOL_NAME = "t_SprayingTool"
WORK_OBJECT = "wobj_SprayingNet"
SPRAY_OUTPUT = "ABB_Scalable_IO_0_DO1"
```

These names must match the ABB controller configuration.

| Parameter | Purpose |
|---|---|
| `HOME_CONFIG` | Joint configuration used before and after spraying |
| `TOOL_NAME` | ABB tool-data name for the spraying tool |
| `WORK_OBJECT` | ABB work-object name for the spraying surface |
| `SPRAY_OUTPUT` | ABB digital output controlling the spray valve |

### Movement parameters

```python
HOME_SPEED = 300
APPROACH_SPEED = 200
TOOLPATH_SPEED = 600
SAFE_OFFSET = 200.0
TOOLPATH_OFFSET = Vector(-400.0, 0.0, 0.0)
```

| Parameter | Current value | Purpose |
|---|---:|---|
| `HOME_SPEED` | 300 mm/s | Joint movement to and from home |
| `APPROACH_SPEED` | 200 mm/s | Movement to safe and first frames |
| `TOOLPATH_SPEED` | 600 mm/s | Linear movement during spraying |
| `SAFE_OFFSET` | 200 mm | Global Z distance above the path |
| `TOOLPATH_OFFSET` | `(-400, 0, 0)` mm | Translation applied to all imported frames |

The safe frame is generated by translating a toolpath frame in global positive Z:

```python
safe_frame.translated(Vector(0, 0, SAFE_OFFSET))
```

The complete imported toolpath is translated using:

```python
toolpath_frames = [
    frame.translated(TOOLPATH_OFFSET)
    for frame in toolpath_frames
]
```

### Select the input file

The current script loads:

```python
data_file = Path(__file__).resolve().parent / "data" / "toolpath_line.json"
```

To use another path, replace the filename, for example:

```python
data_file = Path(__file__).resolve().parent / "data" / "toolpath_circle.json"
```

## 7. Fixed Nozzle Orientation

After the robot reaches `HOME_CONFIG`, the script reads the current TCP frame:

```python
home_frame = abb.send_and_wait(rrc.GetFrame())
```

It then preserves every imported frame position but replaces its orientation with the TCP orientation measured at home:

```python
frames = apply_fixed_orientation(frames, home_frame)
```

This means:

- Grasshopper controls the toolpath positions.
- The current TCP orientation at `HOME_CONFIG` controls the nozzle orientation.
- The nozzle keeps the same orientation throughout the path.

Before running the complete path, confirm that the nozzle orientation at home is correct for spraying.

## 8. Test Stages

Select the stage with:

```python
TEST_STEP = 5
```

| Step | Action |
|---:|---|
| `0` | Test Python-to-ABB communication |
| `1` | Read current robot joints and external axes |
| `2` | Set the tool/work object, move home, and capture TCP orientation |
| `3` | Move to the safe frame above the first path point |
| `4` | Move to the first toolpath frame |
| `5` | Turn on spraying, execute the complete path, retract, and return home |

For a new setup or path, test sequentially:

```text
0 → 1 → 2 → 3 → 4 → 5
```

Do not start with Step 5 until the previous stages have been verified physically.

## 9. Run the Robot Program

Confirm that:

1. Docker services are running.
2. The ABB controller is reachable.
3. The correct tool and work object exist.
4. The digital output name is correct.
5. The robot workspace is clear.
6. The desired JSON file is selected.
7. `TEST_STEP` is set appropriately.

Activate the environment:

```bash
conda activate compas_rrc
```

Run the script from the repository root:

```bash
python 302_send_to_robot.py
```

The script connects to:

```python
abb = rrc.AbbClient(ros, "/rob1")
```

The `/rob1` namespace must match the namespace in `docker/docker-compose.yml`.

## 10. Complete Movement Sequence

With `TEST_STEP = 5`, the program performs:

1. Connect to ROS and ABB.
2. Verify communication.
3. Read the current robot joints.
4. Set `t_SprayingTool`.
5. Set `wobj_SprayingNet`.
6. Set acceleration to `20, 20`.
7. Move to `HOME_CONFIG`.
8. Read the TCP orientation at home.
9. Apply that orientation to all toolpath frames.
10. Move to the first safe frame.
11. Move to the first toolpath frame.
12. Turn `ABB_Scalable_IO_0_DO1` ON.
13. Stream the intermediate frames with linear blended movement.
14. Stop precisely at the final frame.
15. Turn the spray output OFF.
16. Move to a safe frame above the final point.
17. Return to `HOME_CONFIG`.

## 11. Continuous Toolpath Motion

Intermediate targets use:

```python
abb.send(
    rrc.MoveToFrame(
        frame,
        speed=TOOLPATH_SPEED,
        zone=rrc.Zone.Z10,
        motion_type=rrc.Motion.LINEAR,
    )
)
```

This provides:

- Linear TCP movement
- A `Z10` blending zone
- Queued commands through `abb.send(...)`
- Smoother motion without a complete stop at every intermediate frame

The final target uses:

```python
abb.send_and_wait(
    rrc.MoveToFrame(
        frame,
        speed=TOOLPATH_SPEED,
        zone=rrc.Zone.FINE,
        motion_type=rrc.Motion.LINEAR,
    )
)
```

This makes the robot reach and stop at the exact final frame before the spraying output is turned off.

## 12. Spray Output Control

The valve is enabled using:

```python
rrc.SetDigital(SPRAY_OUTPUT, 1)
```

and disabled using:

```python
rrc.SetDigital(SPRAY_OUTPUT, 0)
```

The complete movement loop is wrapped in `try/finally`, so the script attempts to turn the spray output off even if a robot movement raises an error:

```python
try:
    spray_on(abb)
    # robot movement
finally:
    spray_off(abb)
```

This is a software safeguard only. The pneumatic system must still have appropriate physical safety controls.

## Troubleshooting

### Conda environment already exists

```bash
conda env update -f env_compas_rrc.yml --prune
```

### `ModuleNotFoundError`

Activate the correct environment:

```bash
conda activate compas_rrc
```

Then test:

```bash
python -c "import compas_rrc; print(compas_rrc.__version__)"
```

### Docker services do not connect

Check the running containers:

```bash
docker compose -f docker/docker-compose.yml ps
```

View logs:

```bash
docker compose -f docker/docker-compose.yml logs
```

Confirm that:

- Docker Desktop is running.
- Port `9090` is available.
- The ABB controller is reachable at the configured IP address.
- The robot streaming and state ports match the controller setup.
- The Docker namespace is `rob1`.

### JSON file is not found

The data folder must be beside the Python script:

```text
sandskript_compas_rrc/
├── 302_send_to_robot.py
└── data/
    └── toolpath_line.json
```

### JSON does not contain frames

The root JSON object must contain:

```json
{
  "frames": []
}
```

### Toolpath contains no frames

Regenerate and export the path from Grasshopper. The program intentionally stops when the `frames` list is empty.

### Robot moves to the wrong location

Check:

- `WORK_OBJECT`
- `TOOLPATH_OFFSET`
- JSON coordinates
- ABB work-object calibration
- Millimetre units
- First and last frames printed by the script

### Nozzle points in the wrong direction

The script replaces the JSON orientation with the TCP orientation captured at `HOME_CONFIG`. Move only after verifying the home nozzle orientation, tool data, and work object.

### Robot pauses between points

Confirm that intermediate targets use:

```python
abb.send(...)
zone=rrc.Zone.Z10
motion_type=rrc.Motion.LINEAR
```

The final frame should remain `FINE` and use `send_and_wait(...)`.

### Spray does not activate

Check:

- `SPRAY_OUTPUT` matches the ABB signal name exactly.
- The output is configured and writable in RobotWare.
- The valve wiring and power supply are correct.
- The pneumatic supply is connected.
- The robot safety state permits output activation.

## Safety Checklist

Before every physical run:

- [ ] Correct tool mounted and secured
- [ ] Correct ABB tool data selected
- [ ] Correct work object selected and calibrated
- [ ] Correct digital output confirmed
- [ ] Air pressure and valve operation checked separately
- [ ] Toolpath file and offset verified
- [ ] First and last frames inspected
- [ ] Home orientation verified
- [ ] Test Steps 0–4 completed successfully
- [ ] Robot speed reduced for the first run
- [ ] Workspace clear of people and obstacles
- [ ] Hose cannot catch, pull, or enter robot joints
- [ ] Emergency stop accessible
- [ ] Spray output confirmed OFF before approaching the robot

## Current Capabilities

- Grasshopper-to-COMPAS toolpath export
- COMPAS JSON frame loading
- Toolpath validation
- Global XYZ toolpath translation
- Fixed nozzle orientation captured from the robot
- Staged robot testing
- Safe approach and retraction frames
- Continuous blended linear motion
- ABB digital output control
- Automatic spray-off attempt after movement errors
- Return to home after completing the path
- Docker-based ROS, rosbridge, and ABB driver setup

## Project

**Sandskript**  
MAS Architecture and Digital Fabrication  
ETH Zürich  
2026
