# Motor Testing Guide

This guide explains how to test individual motors on each subsystem for hardware validation and troubleshooting.

---

## Purpose
Individual motor testing allows you to:
- Verify correct motor wiring and CAN IDs
- Check motor direction (forward/reverse)
- Validate encoder functionality
- Troubleshoot specific motors without running the full subsystem

---

## How to Use Test Commands

### In RobotContainer.java
Add test bindings in the `configureBindings()` method. Example:

```java
// Test Launcher leader motor at 10 RPS
operatorJoystick.povUp().whileTrue(launcher.testShooterLeaderCommand(10.0));

// Test IntakeFloor motor at 20 RPS
operatorJoystick.povDown().whileTrue(intakeFloor.testMotorCommand(20.0));
```

### Via Shuffleboard/SmartDashboard
You can also trigger these commands through the dashboard for easier testing.

---

## Available Test Commands

### Launcher (4 motors total)
The Launcher has **3 shooter motors** and **1 hood motor**:

```java
// Test shooter LEADER motor only (ID 9)
launcher.testShooterLeaderCommand(rps)

// Test shooter FOLLOWER 1 motor only (ID 10)
launcher.testShooterFollower1Command(rps)

// Test shooter FOLLOWER 2 motor only (ID 11)
launcher.testShooterFollower2Command(rps)

// Test hood motor only (ID 12)
launcher.testHoodCommand(targetDegrees)
```

**Note**: When testing individual shooter motors, the others will stop. This isolates each motor for wiring/direction verification.

---

### LaunchFeeder (1 motor)
```java
// Test feeder motor at specific RPS (CAN ID 13)
launchFeeder.testMotorCommand(rps)
```

---

### FloorFeeder (1 motor)
```java
// Test feeder motor at specific RPS (CAN ID 14)
floorFeeder.testMotorCommand(rps)
```

---

### IntakePivot (1 motor)
```java
// Test pivot motor at specific RPS (CAN ID 15)
intakePivot.testMotorCommand(rps)
```

**Note**: This tests velocity control. For position testing, use the existing `setAngleCommand()`.

---

### IntakeFloor (1 motor)
```java
// Test intake motor at specific RPS (CAN ID 16)
intakeFloor.testMotorCommand(rps)
```

---

### Climber (1 motor)
```java
// Test climber motor at percent output -1.0 to 1.0 (CAN ID 16)
climber.testMotorCommand(percentOutput)
```

**Note**: Climber uses **percent output** (-1.0 to 1.0), not RPS like other subsystems.

---

## Testing Workflow

### Step 1: Identify Motor to Test
Use the Phoenix Tuner or REV Hardware Client to identify which motor you need to test.

### Step 2: Add Test Binding
In `RobotContainer.java`, bind the test command to a button:

```java
private void configureBindings() {
    // ... existing bindings ...
    
    // HARDWARE TESTING - Use POV buttons
    operatorJoystick.povUp().whileTrue(launcher.testShooterLeaderCommand(15.0));
    operatorJoystick.povRight().whileTrue(intakeFloor.testMotorCommand(20.0));
    operatorJoystick.povDown().whileTrue(climber.testMotorCommand(0.3));
}
```

### Step 3: Deploy and Test
1. Deploy code
2. Enable the robot in **Test mode**
3. Hold the test button
4. Observe motor behavior

### Step 4: Verify
- ✅ Motor spins
- ✅ Direction is correct (positive = expected direction)
- ✅ No unusual sounds or vibrations
- ✅ Encoder values change (if applicable)

---

## Safety Notes

> [!CAUTION]
> - Always test in a safe environment with adequate clearance
> - Use low speeds initially (10-20 RPS for velocity motors, 0.2-0.3 for climber)
> - Have E-Stop ready
> - Check for mechanical interference before testing

> [!IMPORTANT]
> - Test commands bypass normal safety interlocks
> - Only use for initial hardware validation
> - Remove test bindings before competition

---

## Troubleshooting

### Motor doesn't spin
1. Check CAN ID in `Constants.java` matches actual hardware
2. Verify motor is powered (check breakers)
3. Check CAN bus wiring
4. Use Phoenix Tuner to verify motor appears on CAN bus

### Motor spins wrong direction
- For TalonFX motors: Change `InvertedValue` in subsystem configuration
- For followers: Change `MotorAlignmentValue` (Aligned ↔ Opposed)

### Encoder doesn't move
1. Verify encoder is properly connected
2. Check `SensorToMechanismRatio` configuration
3. Use Phoenix Tuner self-test

---

## Summary Table

| Subsystem | Motor Count | Test Method | Input Type | CAN IDs |
|-----------|-------------|-------------|------------|---------|
| Launcher | 4 | `testShooterLeaderCommand()` | RPS | 9 |
| | | `testShooterFollower1Command()` | RPS | 10 |
| | | `testShooterFollower2Command()` | RPS | 11 |
| | | `testHoodCommand()` | Degrees | 12 |
| LaunchFeeder | 1 | `testMotorCommand()` | RPS | 13 |
| FloorFeeder | 1 | `testMotorCommand()` | RPS | 14 |
| IntakePivot | 1 | `testMotorCommand()` | RPS | 15 |
| IntakeFloor | 1 | `testMotorCommand()` | RPS | 16 |
| Climber | 1 | `testMotorCommand()` | Percent (-1 to 1) | 16 |

**Total Motors**: 10
