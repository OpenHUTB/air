# Underwater Remotely Operated Vehicle (ROV) Setup and Verification Guide (UE 4.26)

This document describes how to configure, build, and run the underwater remotely operated vehicle (ROV) simulation in AirSim, and how to verify automated control using the Python API.

---

## 1. Overview and Physics Model

AirSim provides vehicle support for underwater and marine robotics (ROVs / AUVs), inspired by UNav-Sim dynamics (defaulting to the BlueROV2 Heavy 8-thruster architecture):
* **Hydrodynamics & Physics Model**: Implemented in `AirLib/include/vehicles/rov/`, providing full 6-DOF marine dynamics:
  * Fluid buoyancy (computed from water density $\rho = 1028\,\text{kg/m}^3$ and vehicle displacement volume);
  * Non-linear fluid drag, quadratic damping, and restoring moments (buoyancy vs. gravity centers);
  * 8-thruster spatial force allocation via a thruster mixer matrix.
* **Firmware (`rov_simple`)**: Provides onboard depth holding, attitude leveling, and body-frame velocity control.
* **Unreal Engine Rendering**: Integrated via native C++ class `ARovPawn`, supporting 5 onboard camera mounts (front right, front left, front center, back center, bottom) and animated thruster rotation.

---

## 2. Prerequisites

* **Operating System**: Windows 10/11 64-bit or Linux (Ubuntu 20.04 / 22.04)
* **Unreal Engine**: Unreal Engine 4.26
* **Compiler Toolchain**:
  * Windows: Visual Studio 2019 (MSVC v142 C++ build tools)
  * Linux: GCC 9 / Clang 11+, CMake 3.19+
* **Python Environment**: Python 3.8 ~ 3.11 (`msgpack-rpc-python`, `numpy`)

---

## 3. Configuration (`settings.json`)

To run AirSim in ROV mode, configure your `settings.json` accordingly.

### 3.1 Example Configuration
```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Rov",
  "ClockSpeed": 1,
  "PawnPaths": {
    "DefaultQuadrotor": {"PawnBP": "Class'/AirSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'"},
    "DefaultRov": {"PawnBP": "Class'/Script/AirSim.RovPawn'"},
    "DefaultComputerVision": {"PawnBP": "Class'/AirSim/Blueprints/BP_ComputerVisionPawn.BP_ComputerVisionPawn_C'"}
  },
  "Vehicles": {
    "RovSimple": {
      "VehicleType": "RovSimple",
      "DefaultVehicleState": "Armed",
      "PawnPath": "DefaultRov",
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "RC": {
        "RemoteControlID": 0,
        "AllowAPIWhenDisconnected": false
      },
      "Cameras": {
        "front_center_custom": {
          "CaptureSettings": [
            {
              "PublishToRos": 1,
              "ImageType": 0,
              "FOV_Degrees": 90,
              "Width": 1280,
              "Height": 720
            }
          ],
          "X": 0.50, "Y": 0.00, "Z": 0.00,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        }
      }
    }
  }
}
```

### 3.2 Configuration File Locations
AirSim searches for `settings.json` in the following order:
1. Command line argument: `-settings="path/to/settings.json"`
2. Unreal project root directory (e.g. `Unreal/Environments/Blocks/settings.json`)
3. User Documents directory: `~/Documents/AirSim/settings.json` (`C:\Users\<username>\Documents\AirSim\settings.json` on Windows)

---

## 4. Building and Running (Blocks Project)

Using the built-in `Blocks` project as an example:

### Step 1: Build the plugin
In the `air` root directory:
* **Windows**:
  ```cmd
  build.cmd
  ```
* Sync the plugin to the `Blocks` environment:
  ```cmd
  cd Unreal\Environments\Blocks
  update_from_git.bat
  ```

### Step 2: Launch the simulation
* Open `Unreal/Environments/Blocks/Blocks.uproject` in Unreal Engine 4.26;
* Click the **Play** button in the editor toolbar;
* The ROV will spawn in the viewport. The log outputs:
  ```text
  LogTemp: StartupModule: AirSim plugin
  LogTemp: ARovPawn: Loaded ROV mesh from /AirSim/Models/RoV/...
  LogTemp: RovSimple
  LogTemp: SimModeWorldRov: Api server started on port 41451
  ```

---

## 5. Python API Verification

While the simulation is running:

### Step 1: Install Python client
```bash
cd PythonClient
pip install -e .
```

### Step 2: Run verification script
```bash
python PythonClient/rov/hello_rov.py
```

### Step 3: Expected output
```text
Connecting to AirSim ROV...
Connected!
Client Ver:1 (Min Req: 1)
Server Ver:1 (Min Req: 1)

Arming the ROV...

ROV State:
  Kinematics Position: x=0.00, y=0.00, z=0.00
  Kinematics Orientation: w=1.00, x=0.00, y=0.00, z=0.00
  Linear Velocity: vx=0.00, vy=0.00, vz=0.00

IMU Data:
  Linear Acceleration: (0.00, 0.00, -9.81) m/s^2
  Angular Velocity: (0.00, 0.00, 0.00) rad/s

Barometer (Depth) Data:
  Altitude: 0.00 m
  Pressure: 101325.0 Pa

Magnetometer Data:
  Magnetic Field: (0.21, 0.02, 0.42) Gauss

Moving ROV by velocity in body frame (forward 0.5 m/s, dive 0.2 m/s for 3s)...
Updated ROV Position: x=1.48, y=0.00, z=0.59

Hovering...
Disarming and resetting...
Done!
```

---

## 6. Troubleshooting

1. **Connection refused on port 41451**:
   - Ensure the Unreal simulation is actively running (Play mode).
   - Check that `"SimMode": "Rov"` is specified in `settings.json`.
2. **Vehicle not spawning or mesh missing**:
   - Verify that `PawnPaths` in `settings.json` contains `"DefaultRov": {"PawnBP": "Class'/Script/AirSim.RovPawn'"}`.
