# Intelligent Overhead Crane

Academic robotics project developed by a 14-member EMINES–UM6P team in collaboration with Sonasid. The prototype combines three translational axes with load rotation for material handling.

**Portfolio of El Mehdi Sahhar.** My contribution focused on control and anti-sway modeling/input shaping. This repository also includes shared team components for perception and the operator interface; it does not imply sole authorship of the full system.

## What is implemented

- **Anti-sway control:** the Arduino sketch implements a two-impulse ZV filter on X/Y velocity commands. Its delay is updated from the effective cable length, `T = pi * sqrt(L_eff / g)`, using equal impulse weights of 0.5. Z uses a trapezoidal profile. No swing-angle measurement is required by this feedforward shaper.
- **Waypoint planning:** the Python/Tkinter controller builds a visibility graph around expanded rectangular obstacles and uses Dijkstra's algorithm. Low obstacles can be overflown subject to the configured vertical workspace. The interface previews paths and exchanges commands with the Arduino over serial.
- **Perception:** YOLO detections, stereo rectification and disparity-based depth (`Z = baseline * fx / disparity`), with experimental XY-coordinate transformations.
- **Actuation:** step/direction motor control, homing switches, electromagnet and rotation commands.

The broader project explored ZV/ZVD input shaping. **The supplied firmware implements ZV, not ZVD.** Approximately 80% oscillation reduction was reported by El Mehdi for project simulations. The supplied archive does not include the simulation benchmark needed to reproduce that percentage, so it is not presented as a measured hardware result.

## Repository guide

| File | Purpose |
| --- | --- |
| `controleur_waypoints.py` | Operator GUI, serial communication, waypoint planning and mission sequencing |
| `pont_roulant_3axes (1).ino` | Arduino motion controller with ZV shaping |
| `code_detection.py` | Multi-model stereo object detection experiment |
| `EssaiXY.py`, `Essai2` | Experimental variants for position estimation |
| `calibration.py`, `triangulation.py` | Rectification, calibration loading and depth calculation |
| `stereo_calibration` | Original calibration script (Python, no extension) |
| `calibration_images.py`, `2camsouv.py` | Calibration capture and camera checks |
| `stereoMap.xml` | Supplied calibration maps; specific to the original camera setup |
| `best (2).pt`, `best (3) (1).pt`, `yolov8n.pt` | Model weights included in the archive |
| `code detection toy truck.ipynb` | Training notebook with outputs and metadata cleared |
| `toy_truck_fardeaux.py` | Standalone training example adapted from the Colab export |

## Setup

Use Python 3.10 or later and a Python installation with Tkinter:

```bash
python -m venv .venv
# Activate the environment using your operating system's command.
pip install -r requirements.txt
python controleur_waypoints.py
```

The GUI opens without automatically connecting to the controller. Hardware operation requires the correct serial port, wiring, motor settings and workspace parameters.

For Arduino, install the board toolchain and the **MX_AX_Serial1** library, which is referenced by the sketch but is **not included**. Create a sketch folder whose name matches the `.ino` file as required by the Arduino IDE. Verify pin assignments and actuator limits before connecting the prototype.

## Perception and training prerequisites

- Run perception scripts from the repository directory so relative model paths resolve.
- The experiments reference `best (1).pt` (person) and `best (5).pt` (box), which were **not present in the archive**. Supply those models or deliberately adapt the model configuration before running these experiments.
- Recalibrate for your own cameras. The original 35 left/right calibration image pairs are not included in this portfolio release; use `calibration_images.py` to collect your own set.
- YOLO training requires authorized access to the original Roboflow dataset. Set `ROBOFLOW_API_KEY` in your local environment; never commit it.
- The notebook retains its Colab-specific workflow. The standalone training example removes Drive mounting and shell-magics from the Python export.

## Scope and validation

This is an academic prototype, not a certified industrial safety controller. The geometric planner assumes the obstacle positions supplied to it; the archive does not establish an automatic end-to-end connection between the perception scripts and the GUI. No formal collision-avoidance or stability guarantee is claimed.

Publication preparation checked Python syntax and removed embedded API keys, personal absolute paths, notebook outputs and caches. No hardware motion, firmware upload, camera experiment, model training or trained-weight loading was performed during that preparation. Dependencies are listed without a claimed, tested version lock.

## Provenance

Prepared from the supplied `pont_roulant.zip`. Original control and perception algorithms are retained. Changes for publication: relative model paths, sanitized notebook, environment-based credential handling, a cleaned training entry point and documentation. Calibration photographs and Python bytecode caches were omitted.

No new license is assigned to the team code or supplied weights. Original contributors' and third-party rights remain applicable; consult their licenses before redistribution or reuse.
