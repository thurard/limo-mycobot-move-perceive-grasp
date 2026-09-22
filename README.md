# limo-mycobot-move-perceive-grasp

A **"move – perceive – grasp"** robotics pipeline developed during my
internship at the **Department of Production and Robotics Engineering, KMUTNB
(Bangkok)**, from May to August 2026.

The system combines an **AgileX LIMO** mobile robot (autonomous navigation +
perception) with a **myCobot 280 Pi** arm (object grasping), linked over a
network bridge. The goal: have the LIMO patrol a mapped area, detect objects,
and trigger the grasping of a target by the arm.

> **Context.** This repository gathers the internship code exactly as it ran on
> the real hardware. It is not a packaged ROS2 (`colcon`) package, but a set of
> scripts launched by hand on each machine. The scripts are heavily commented
> (in French) to stay readable and reusable.

---

## Hardware architecture

```
   ┌──────────────────────────┐         Wi-Fi network (iPhone hotspot)       ┌───────────────────────────┐
   │        AgileX LIMO        │   ─────────────  TCP socket  ─────────────►  │   myCobot Raspberry Pi     │
   │  (ROS2 Foxy)              │            JSON / port 5555                   │   (myCobot 280 Pi)         │
   │                           │                                              │                            │
   │  • Nav2 (AMCL, DWB)       │                                              │  • pymycobot               │
   │  • Cartographer (SLAM)    │                                              │  • OpenCV (gripper camera) │
   │  • Orbbec RGB-D camera    │                                              │  • visual servoing         │
   │  • YOLOv8n (detection)    │                                              │    in joint angles         │
   │                           │                                              │                            │
   │  client_bras.py  ────────────────► serveur_bras.py  (listens on 5555)   ◄──────────────────────────┘
   └──────────────────────────┘
```

- **LIMO**: navigation, mapping, global perception (YOLO + depth).
- **Arm Pi**: receives commands from the LIMO/PC through a socket server and
  performs the grasp with its own camera embedded in the gripper.
- **Network bridge**: `client_bras.py` (LIMO/PC side) ↔ `serveur_bras.py` (Pi
  side), JSON messages over port **5555**.

---

## Key technical result: visual servoing *in joint angles*

The hard part of the internship was reliably grasping the target. The myCobot's
**Cartesian** command (`send_coords`) saturates silently near the joint limits:
the arm "accepts" the setpoint without reaching it and ends up in a degenerate
pose. The chosen solution is **servoing in joint angles**
(`guidage_angles.py`):

- **J1 (base)** and **J2 (shoulder)** are corrected directly from the pixel
  error between the detected target and an image setpoint (`CIBLE_DX`,
  `CIBLE_DY`), using an empirical Jacobian;
- **step-by-step descent** (`PALIER_MM`) with **re-centering** at each step;
- **gripper protection**: closing at `VALEUR_FERMETURE = 50` (not 255, which
  forces and damages the servo), with a power-cycle reset.

This joint-angle approach is what made the grasp robust and repeatable.

---

## Directory layout

```
limo-mycobot-move-perceive-grasp/
├── limo/                     # code run ON the LIMO
│   ├── patrouille.py          # patrol node: Nav2 navigation + YOLO + report
│   ├── client_bras.py         # sends commands to the arm (socket, port 5555)
│   ├── limo_start.launch.py   # static TFs (laser, camera, IMU) + robot description
│   ├── pub_robot_description.py
│   ├── start_patrol.sh        # patrol launch script
│   └── start_mapping.sh       # mapping launch script (Cartographer)
│
├── arm/                      # code run ON the myCobot Raspberry Pi
│   ├── serveur_bras.py        # socket server: receives actions and executes them
│   ├── guidage_angles.py      # ★ visual servoing IN JOINT ANGLES (reliable grasp)
│   ├── saisie.py              # grasp/drop/rest poses by coordinates
│   ├── deposer.py             # drop at a fixed position
│   ├── detect_couleur.py      # colored-object detection (gripper camera, HSV)
│   ├── detect_3d.py           # YOLO + depth → 3D position (camera frame)
│   ├── detect_photo.py        # detection on a still photo (offline)
│   ├── calib_couleur.py       # HSV range calibration
│   ├── calib_axes.py          # image axes ↔ arm motion calibration
│   └── diag_camera.py         # camera diagnostics (dimensions, alignment, cloud)
│
├── config/                  # ROS2 / Nav2 / SLAM configuration files
│   ├── nav2_src.yaml          # "source" Nav2 config (inflation_radius 0.45)
│   ├── nav2_install.yaml      # "installed" Nav2 config (footprint scale 10.0,
│   │                          #   camera in obstacle_layer via pointcloud)
│   └── limo_lds_2d.lua        # Cartographer config (tracking_frame=base_link, IMU)
│
├── maps/                    # map used by the patrol
│   ├── map_bras.pgm
│   └── map_bras.yaml
│
└── archive/                 # obsolete versions kept for history
    └── guidage_visuel.py      # old CARTESIAN servoing (dropped:
                               #   silent saturation of send_coords)
```

---

## Launch (actual order used during the internship)

The scripts run on **two machines**; the order below is the one actually
followed during the demonstrations.

**1. On the arm's Raspberry Pi** — start the server:
```bash
python3 ~/serveur_bras.py
```
The server listens on port 5555 and waits for commands.

**2. On the LIMO** — start the camera, localization (Nav2/AMCL), then the
patrol:
```bash
./start_patrol.sh      # or launch patrouille.py manually
```
The LIMO moves through the map's waypoints, saves a photo of each point at
~1.5 m, and generates a CSV + HTML report of the detections.

**3. Triggering the grasp** — from the LIMO (or the PC), when the robot is in
position facing the target:
```bash
python3 client_bras.py saisir_vision   # the arm searches for the ball and grasps it
python3 client_bras.py deposer_fixe    # the arm drops it at the fixed position
```

### Patrol ↔ arm integration

The orchestration between the patrol and the arm was **semi-automatic**, not
fully autonomous end-to-end:

- `patrouille.py` handles navigation and perception on its own.
- At the point where the arm should act, `patrouille.py` contains **a
  placeholder** (`print(...)` + `time.sleep(2)`): **it does not yet
  automatically call** `client_bras.py`.
- In practice, `serveur_bras.py` was started *before* the patrol, then the
  `saisir_vision` / `deposer_fixe` commands were **sent manually** while the
  LIMO patrolled autonomously.

The network bridge and the grasp are therefore fully functional and tested; what
remains is to replace the placeholder in `patrouille.py` with a real call to
`commander(...)` (already importable from `client_bras.py`) to close the loop
automatically.

---

## Arm server actions

`serveur_bras.py` accepts the following JSON actions (via `client_bras.py`):

| Action            | Effect                                                     |
|-------------------|------------------------------------------------------------|
| `etat`            | returns the arm's current state                            |
| `ranger`          | folds the arm back to its rest pose                        |
| `saisir_vision`   | autonomous grasp via visual servoing in joint angles       |
| `deposer_fixe`    | drop at the predefined fixed position                      |
| `saisir` (coords) | grasp at given coordinates                                 |
| `deposer` (coords)| drop at given coordinates                                  |

---

## Environment

- **ROS2 Foxy**, Nav2 (AMCL, DWB, costmaps), Cartographer (SLAM), CycloneDDS.
- **Perception**: YOLOv8n (`yolov8n.pt`, COCO classes), OpenCV, Orbbec RGB-D
  camera.
- **Arm**: `pymycobot`, myCobot 280 Pi.
- The YOLO weights (`*.pt`) are **not** versioned (see `.gitignore`): download
  them separately.

---

## Author

**Tom Hurard** — engineering student at SeaTech (Toulon), SYSMER track.
Internship supervised by **Amornphun Phunopas** (KMUTNB) and **Cédric
Anthierens** (SeaTech).
