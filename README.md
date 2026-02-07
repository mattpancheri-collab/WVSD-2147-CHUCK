# 2026 Robot Code - Student Guide

Welcome to the 2026 Robot Code project! This codebase has been structured to make it easy for you to learn, adjust, and program the robot.

## 🚀 Getting Started

1.  **Open the Project**: Open this folder in VS Code with the WPILib extension installed.
2.  **Build the Code**: Open the command palette (Ctrl+Shift+P) and type `WPILib: Build Robot Code`.

## 📂 Project Structure

-   `src/main/java/frc/robot/`
    -   `subsystems/`: Contains the code for each physical mechanism (Launcher, Intake, Climber, etc.).
    -   `Constants.java`: The central place for **CAN IDs** and **Controller Ports**.
    -   `RobotContainer.java`: The central place for **Button Bindings** and connecting subsystems.

## 🛠️ How to Make Adjustments

We have marked key areas for you to edit with `TODO: STUDENTS` or `STUDENT ADJUSTMENT AREA`.

### 1. Changing CAN IDs
If you change a motor controller on the robot, you need to update its ID here:
-   **File**: `Constants.java`
-   **Look for**: `MechanismIds` class.

### 2. Tuning Speeds & PID
Each subsystem has configuration constants at the top of its file.

**For Velocity-Controlled Subsystems** (Launcher, Feeders, IntakeFloor):
-   **Files**: `subsystems/LaunchFeeder.java`, `subsystems/FloorFeeder.java`, etc.
-   **Target Speed Constants**: `FEED_IN_RPS`, `IN_RPS`, `kTargetRPM`
-   **PID Constants**: `kP`, `kI`, `kD` (Start with kP, leave kI and kD at 0)
-   **Tuning Tip**: Increase kP until the motor reaches target speed quickly without oscillation

**For Position-Controlled Subsystems** (IntakePivot):
-   **File**: `subsystems/IntakePivot.java`
-   **Position Presets**: `STOW_DEG`, `AMP_DEG`
-   **PID Constants**: `kP`, `kG` (gravity compensation)
-   **Feedforward**: `kV`, `kA` for smooth motion

### 3. Binding Buttons
To make a button do something (like run the intake), go to the container.
-   **File**: `RobotContainer.java`
-   **Look for**: `configureBindings()` method.
-   **Example**: `operatorJoystick.a().whileTrue(launcher.runLauncherCommand());`

## 🧩 Subsystems Overview

| Subsystem | Description | Control Type |
| :--- | :--- | :--- |
| **Launcher** | Dual-motor shooter with velocity PID control | Velocity (RPS) |
| **LaunchFeeder** | Feeder with CANrange sensor for game piece detection | Velocity (RPS) |
| **FloorFeeder** | Intake feeder with current limiting and slew rate control | Velocity (RPS) |
| **IntakePivot** | Position-controlled pivot with gravity compensation | Position (Degrees) |
| **IntakeFloor** | Floor intake roller with velocity control | Velocity (RPS) |
| **Climber** | Simple climber with brake mode enabled for safety | Percent Output |

### Advanced Features
- **PID Control**: Most subsystems use **closed-loop velocity** or **position** control with tunable kP, kI, kD gains
- **Telemetry Logging**: Subsystems use `@Logged` annotations for automatic data logging via WPILib Epilogue
- **Simulation Support**: IntakePivot and FloorFeeder include simulation models for testing without hardware
- **Current Limiting**: All TalonFX motors have stator current limits to prevent brownouts

## 📦 Deployment
1.  Connect to the robot via USB or Wi-Fi.
2.  Open Command Palette (Ctrl+Shift+P).
3.  Run `WPILib: Deploy Robot Code`.
