# 2026 Robot Code - Student Guide

Welcome to the 2026 Robot Code! This guide is designed to help you understand how our robot works, where to find things in the code, and how to safely test and tune the mechanisms.

---

## 🚀 Getting Started

1. **Open the Project**: Open this folder in VS Code. Make sure you launch it using the WPILib icon (or ensuring the WPILib extension is active).
2. **Build the Code**: Open the command palette (`Ctrl+Shift+P`) and type `WPILib: Build Robot Code`. This checks for errors.
3. **Deploy the Code**: Connect to the robot via USB or Wi-Fi. Open the Command Palette (`Ctrl+Shift+P`) and select `WPILib: Deploy Robot Code`.

---

## 🤖 How the Robot Works (Subsystems)

A **Subsystem** is the code that controls a physical part of the robot (like an arm, a shooter, or the wheels). Here are the main parts of our 2026 robot:

*   **CommandSwerveDrivetrain**: Controls the wheels (driving and turning). We use Phoenix 6 and PathPlanner for this.
*   **Launcher**: The dual-motor shooter that launches game pieces. It uses *Velocity* control (getting wheels to exact speeds).
*   **LaunchFeeder & FloorFeeder**: The rollers that physically move the game piece through the robot and up into the shooter.
*   **IntakeGround**: The roller that picks pieces up off the floor.
*   **IntakePivot**: The motor that swings the intake up (for stowing) or down (for picking up). This uses *Position* control (moving to exact angles).
*   **Climber**: A simple motor that pulls the robot up at the end of the match.

All of these files live in: `src/main/java/frc/robot/subsystems/`

---

## 🔎 How to Find Things

We organize our code so there is exactly one place to look for specific details. 

### 1. Where are the Motor IDs? (CAN IDs)
If you replace a motor or add a new one, you need to tell the code its new ID on the CAN bus.
*   **File**: `src/main/java/frc/robot/Constants.java`
*   **Where to look**: The `CANConstants` class at the top of the file.

### 2. Where are the Controller Buttons?
If you want to make a button do something (like "Press A to shoot"), you bind the button to a Command.
*   **File**: `src/main/java/frc/robot/RobotContainer.java`
*   **Where to look**: Scroll down to the `configureBindings()` method. You will see sections for `DRIVER (SWERVE)` and `OPERATOR`.

---

## 🛠️ How to Tune and Make Adjustments

Tuning is the process of adjusting the code so the robot moves flawlessly. You will mostly be adjusting speeds (RPS/RPM) and PID gains (math values that help the motor reach its target).

All tuning values are located in **`Constants.java`**.

### Tuning a Velocity Mechanism (Shooter or Feeders)
For things that spin continuously, we tune Velocity.
1. **Find the Constants**: Open `Constants.java` and look for `LauncherConstants`, `LaunchFeederConstants`, etc.
2. **Change the Target Speed**: Adjust variables like `kShooterVolts` or `kFeedOutRPS`.
3. **Change the PID (If needed)**: If the wheel takes too long to spin up, or if it overshoots its target, adjust `kP`, `kI`, or `kD`. (Always start with `kP` and keep the others at 0 until needed).

### Tuning a Position Mechanism (Intake Pivot)
For things that need to stop at an exact angle, we tune Position.
1. **Find the Constants**: Open `Constants.java` and look for `IntakePivotConstants`.
2. **Change the Target Angles**: Look for `kStowAngleDeg` and `kDeployAngleDeg`. Adjust these to change where the intake stops.
3. **Change the PID**: If the arm is too slow, increase `kP`. If it drops due to gravity, check the `kG` (Gravity feedforward) value.

### Testing Modes (Voltage Testing)
To safely test a mechanism before giving it a final speed, we have a "Testing Joystick" configured in `RobotContainer.java`. 
1. Open `Constants.java` and find the `TestingConstants` section.
2. Here, you can set very safe, low voltages (e.g., 2.0 Volts). 
3. Deploy the code and use the designated Testing Controller (Port 2) to move the motors slowly and verify they spin the correct direction.

---

## 🗺️ Autonomous Paths (PathPlanner)

We use an app called **PathPlanner** to draw paths for the robot to follow during the autonomous period.

*   **Where paths are chosen**: In `RobotContainer.java`, the `autoChooser` automatically finds the paths you drew in the app and puts them on the dashboard.
*   **Where actions are mapped**: In `RobotContainer.java`, inside the `registerNamedCommands()` method. **Crucial:** The string name (e.g., `"Shoot"`) in the code *must perfectly match* the name you type into the PathPlanner event marker.
*   **Tuning the Auto**: If the robot drifts off the path, or doesn't turn correctly, you must tune the PathPlanner PID gains. Open `subsystems/CommandSwerveDrivetrain.java` and read the comments around `PPHolonomicDriveController`.
