# 2026 Robot Code

Welcome to the software repository for our 2026 FRC Robot! This repository uses Java, WPILib, standard Phoenix 6 libraries, and PathPlanner to control our custom Swerve Drive and mechanisms.

To better serve our team, documentation has been split into two distinct guides. Please select the guide most applicable to you:

---

## 📘 [Student Guide](Student_Guide.md)
Are you a student learning how to program the robot? Start here! This guide covers:
- How to open the project, build, and deploy to the robot.
- A high-level overview of how the robot's physical parts (subsystems) work.
- Instructions on where to find CAN IDs, bind controller buttons, and tune mechanism speeds.

## 📙 [Mentor Guide](Mentor_Guide.md)
Are you a software mentor guiding students? Read this guide to understand:
- The architectural philosophy of the codebase (Command-based framework, centralized constants, factories).
- Common pitfalls students encounter and how the code is structured to help them.
- Where to find and direct students for advanced Swerve Drive and PathPlanner tuning.

---

## 🔧 Software Requirements
- [VS Code with WPILib extension](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- Phoenix Tuner X
- PathPlanner App

## 🏗️ Repository Structure
- `src/main/java/frc/robot/`
  - `Constants.java`: Central location for all CAN IDs, ports, and tuning offsets.
  - `RobotContainer.java`: Central location for button bindings and PathPlanner named commands.
  - `subsystems/`: Code encapsulating physical hardware (Launcher, Intake, Swerve, etc.).
  - `commands/`: Factories encapsulating complex sequences and auto routines.
