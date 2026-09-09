# 水下机器人 (ROV) 仿真配置与运行指南 (UE4.26)

本文档说明如何在 `air` (AirSim) 项目中配置、编译与运行水下机器人（Remotely Operated Vehicle, ROV）仿真特性，并进行 Python API 自动化控制验证。

---

## 1. 概述与物理模型

AirSim 新增支持了基于 UNav-Sim 动力学模型的水下机器人 (ROV) 载具支持（默认使用 BlueROV2 Heavy 架构）：
* **物理模型**：在 `AirLib/include/vehicles/rov/` 中实现了水动力六自由度方程，包括：
  * 流体浮力（基于设定水密度 $\rho = 1028\,\text{kg/m}^3$ 与排水体积计算）；
  * 非线性水阻力矩与二次流体阻力（Drag factors）；
  * 重力与浮心平衡力矩；
  * 8 推进器空间推力映射（Mixer Matrix）。
* **内置固件**：提供 `rov_simple` 固件，支持深度闭环（Depth Hold）、姿态闭环（Angle/Rate Level）以及体坐标系速度控制。
* **渲染呈现**：在 UE 4.26 中通过原生 C++ 类 `ARovPawn` 实现，包含 5 个机载相机（前右、前左、前中、后中、下置）与推进器旋转。

---

## 2. 环境前置条件

* **操作系统**：Windows 10/11 64-bit 或 Linux (Ubuntu 20.04 / 22.04)
* **虚幻引擎**：Unreal Engine 4.26
* **编译工具链**：
  * Windows：Visual Studio 2019 (安装 MSVC v142 C++ 构建工具)
  * Linux：GCC 9 / Clang 11+, CMake 3.19+
* **Python 环境**：Python 3.8 ~ 3.11（安装 `msgpack-rpc-python`、`numpy`）

---

## 3. 仿真配置文件 (`settings.json`)

仿真运行前，需确保 `settings.json` 设置为 ROV 模式。

### 3.1 推荐配置样例

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

### 3.2 配置文件路径优先级
AirSim 会依次在以下路径查找 `settings.json`：
1. 命令行参数 `-settings="路径"`
2. 项目工程根目录下的 `settings.json`（例如 `Unreal/Environments/Blocks/settings.json`）
3. 用户文档目录：`~/Documents/AirSim/settings.json`（Windows 为 `C:\Users\<用户名>\Documents\AirSim\settings.json`）

---

## 4. 独立工程编译与启动 (Blocks 项目)

以官方自带的 `Blocks` 独立演示项目为例：

### 步骤 1：同步更新插件与编译
在 `air` 仓库根目录下执行构建脚本：
* **Windows**:
  ```cmd
  build.cmd
  ```
  或者在 Visual Studio 2019 中打开 `AirSim.sln`，选择 `DebugGame_Editor` 或 `Development_Editor` 平台编译。
* 将编译好的插件拷贝/同步至 Blocks 环境：
  ```cmd
  cd Unreal\Environments\Blocks
  update_from_git.bat
  ```

### 步骤 2：启动仿真
* 在虚幻引擎中打开 `Unreal/Environments/Blocks/Blocks.uproject`；
* 点击编辑器工具栏的 **Play** 运行；
* 启动成功后，可在视口中看到 BlueROV2 水下机器人载具，日志终端会打印：
  ```text
  LogTemp: StartupModule: AirSim plugin
  LogTemp: ARovPawn: Loaded ROV mesh from /AirSim/Models/RoV/...
  LogTemp: RovSimple
  LogTemp: SimModeWorldRov: Api server started on port 41451
  ```

---

## 5. Python API 自动化控制与验证

在仿真处于运行状态时，可以通过 Python 客户端与 ROV 建立 RPC 连接并发送指令。

### 步骤 1：准备 Python 客户端
进入 `PythonClient` 目录：
```bash
cd PythonClient
pip install -e .
```
（或者运行前设置 `PYTHONPATH` 包含 `PythonClient` 目录）。

### 步骤 2：执行测试脚本
执行内置的自动化测试脚本：
```bash
python PythonClient/rov/hello_rov.py
```

### 步骤 3：验证输出说明
脚本执行时将依次对 ROV 执行以下测试，成功时的标准输出如下：

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

各指标说明：
1. **连接与解锁**：`confirmConnection()` 与 `armDisarm(True)` 成功，表示 41451 RPC 通信畅通且固件推进器解锁。
2. **传感器数据**：
   * IMU 正确反映静止时的重力加速度（NED 坐标系下 Z 轴约 $-9.81\,\text{m/s}^2$）；
   * 深度计与气压计正常反馈水深/气压；
   * 磁力计正确反馈三轴地磁场数据。
3. **六自由度运动响应**：
   * 指令要求以 $v_x = 0.5\,\text{m/s}$ 前进、以 $v_z = 0.2\,\text{m/s}$ 下潜持续 3 秒；
   * 实际位置更新显示 $x \approx 1.5\,\text{m}$，$z \approx 0.6\,\text{m}$，位移精确响应。
4. **定深与悬停**：`hoverAsync()` 指令生效，ROV 在浮力与推进器反作用力下实现原位稳定悬停。

---

## 6. 常见问题排查

1. **端口无法连接 (Connection refused on 41451)**：
   * 确认虚幻编辑器是否已点击 **Play**；
   * 检查 `settings.json` 中 `"SimMode"` 是否被正确设置为 `"Rov"`。
2. **载具未生成或网格丢失**：
   * 检查 `settings.json` 中 `PawnPaths` 是否包含 `"DefaultRov": {"PawnBP": "Class'/Script/AirSim.RovPawn'"}`。
3. **中文路径读取异常**：
   * 本分支已针对 Windows UTF-8 宽字符路径进行编码修复，建议将 `settings.json` 直接放置在虚幻工程目录或文档目录下。
