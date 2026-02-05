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
Each subsystem has specific variables for speeds (RPM, % Output) and PID gains at the top of the file.
-   **Files**: `subsystems/Launcher.java`, `subsystems/IntakeFloor.java`, etc.
-   **Look for**: `kTargetRPM`, `kIntakeSpeed`, etc.

### 3. Binding Buttons
To make a button do something (like run the intake), go to the container.
-   **File**: `RobotContainer.java`
-   **Look for**: `configureBindings()` method.
-   **Example**: `operatorJoystick.a().whileTrue(launcher.runLauncherCommand());`

## 🧩 Subsystems Overview

| Subsystem | Description |
| :--- | :--- |
| **Launcher** | Controls the main shooter wheels. Tune RPMs here. |
| **LaunchFeeder** | Feeds the game piece into the Launcher. |
| **FloorFeeder** | Moves game pieces from the intake to the center. |
| **IntakePivot** | Rotates the intake in and out (Deployed/Stowed). |
| **IntakeFloor** | Spins the intake rollers to grab game pieces. |
| **Climber** | Controls the arms to pull the robot onto the chain. |

## 📦 Deployment
1.  Connect to the robot via USB or Wi-Fi.
2.  Open Command Palette (Ctrl+Shift+P).
3.  Run `WPILib: Deploy Robot Code`.
