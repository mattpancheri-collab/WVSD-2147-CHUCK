# Quality Assurance Report
**QA Engineer #2 - Codebase Analysis**  
**Date**: 2026-02-07  
**Workspace**: `h:/2026_Robot_Code`

---

## Executive Summary
This document identifies **critical mismatches** between documented subsystems and actual implementations, as well as configuration errors that will prevent the code from compiling or functioning correctly for students.

---

## 🔴 CRITICAL ISSUES

### 1. **MAJOR CODE MISMATCH: Subsystems Do Not Match Documentation**
**Severity**: CRITICAL  
**Files Affected**: All subsystems except `Climber.java`

**Issue**:
The subsystem files in the repository (`LaunchFeeder`, `FloorFeeder`, `IntakeFloor`, `IntakePivot`) contain **sophisticated, production-quality code** that does NOT match the simple, student-friendly implementations referenced in:
- `RobotContainer.java` (lines 84-85, 88-89, etc.)
- `README.md`
- `walkthrough.md`

**Evidence**:
- `LaunchFeeder.java` has 214 lines with CANrange sensor integration, auto-stop logic, and epilogue logging
- `RobotContainer.java` expects simple methods like `feedCommand()` but actual file has `feedUntilBallCommand()`, `feederInCommand()`, etc.
- Original design had `kFeedSpeed = 0.5` (percentage), but actual code uses RPS with PID (`kP = 0.12`)

**Impact**:
- Students will be confused by complex code that doesn't match the guide
- Method names don't align (compilation errors possible)
- Advanced features (CANrange sensors, Epilogue logging) require dependencies not mentioned

---

### 2. **CAN ID Conflicts**
**Severity**: HIGH  
**Files**: `Constants.java` vs actual subsystem implementations

| Subsystem | Constants.java | Actual File | Status |
|-----------|---------------|-------------|--------|
| `LaunchFeeder` | ID: 12 | ID: 13 | ❌ MISMATCH |
| `FloorFeeder` | ID: 13 | ID: 14 | ❌ MISMATCH |
| `IntakePivot` | ID: 14 | ID: 15 | ❌ MISMATCH |
| `IntakeFloor` | ID: 15 | ID: 16 | ❌ MISMATCH |
| `Climber` | ID: 16 | Uses `MechanismIds.kClimberID` | ✅ Correct |

**Impact**: Runtime errors when attempting to communicate with incorrect motors.

---

### 3. **Missing Dependencies / Imports**
**Severity**: HIGH  
**Files**: `LaunchFeeder.java`, `FloorFeeder.java`, `IntakeFloor.java`, `IntakePivot.java`

**Missing Classes**:
```java
import com.ctre.phoenix6.hardware.CANrange;  // LaunchFeeder line 7
import edu.wpi.first.epilogue.Logged;         // Used in all advanced subsystems
```

**Issue**: 
- `CANrange` class may not exist in standard Phoenix 6 library (typo for `CANcoder`?)
- `@Logged` annotations require Epilogue library which is NOT in `vendordeps/`

**Impact**: Code will not compile until dependencies are added.

---

## ⚠️ CONFIGURATION ERRORS

### 4. **Inconsistent Naming: `generated` vs `Generated`** 
**Severity**: MEDIUM  
**Files**: `RobotContainer.java` line 15

```java
import frc.robot.generated.TunerConstants;  // lowercase 'g'
```

**Actual Directory**: `src/main/java/frc/robot/Generated/` (capital 'G')

**Impact**: Case-sensitive systems (Linux, macOS deploy) will fail to compile.

---

### 5. **Method Name Mismatches in RobotContainer**
**Severity**: HIGH  
**File**: `RobotContainer.java`

**Line 85**: `operatorJoystick.leftBumper().whileTrue(launchFeeder.feedCommand());`
- **Expected**: `feedCommand()` does exist (line 198 of `LaunchFeeder.java`)
- Status: ✅ OK (surprisingly)

**Line 88**: `operatorJoystick.x().whileTrue(intakeFloor.runIntakeCommand());`
- **Actual method**: `intakeInCommand()` (line 206 of `IntakeFloor.java`)
- Status: ❌ **COMPILATION ERROR**

**Line 89**: `operatorJoystick.b().whileTrue(intakeFloor.outtakeCommand());`
- **Actual method**: `intakeOutCommand()` (line 211 of `IntakeFloor.java`)
- Status: ❌ **COMPILATION ERROR**

---

### 6. **Original Simple Subsystems Are Missing**
**Severity**: CRITICAL (Documentation)  
**Files**: All subsystems except `Climber.java` and `Launcher.java`

**Issue**:
The walkthrough and README reference simple subsystems with:
- Basic `TODO: STUDENTS` comments
- Simple constants like `kFeedSpeed = 0.5`
- Minimal PID configuration

**Actual Code Contains**:
- Production-level velocity PID with slot configs
- Slew rate limiters
- Advanced telemetry (@Logged annotations)
- Simulation support
- Current limiting configurations

**Recommendation**: Either:
1. Revert to simple implementations as documented, OR
2. Update all documentation to reflect advanced implementations

---

## 🟡 MINOR ISSUES

### 7. **Inconsistent Comment Style**
**Files**: `Climber.java` vs other subsystems

- `Climber.java`: Uses old style (lines 17-20) with student TODOs commented out
- Other subsystems: Have explicit adjustment areas with detailed tuning guides

**Recommendation**: Standardize on one approach.

---

### 8. **Unused Imports and Dead Code**
**File**: `IntakePivot.java`

**Line 361-363**:
```java
@SuppressWarnings("unused")
private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
}
```

**Issue**: Method is marked `@SuppressWarnings("unused")` suggesting it's not currently used.

---

### 9. **Missing Error Handling**
**Severity**: MEDIUM  
**Files**: All subsystems

**Issue**: No try-catch blocks or status code validation when applying motor configurations.

**Example** (`LaunchFeeder.java` line 96):
```java
motor.getConfigurator().apply(config);
```

**Recommendation**: Check return status:
```java
StatusCode status = motor.getConfigurator().apply(config);
if (!status.isOK()) {
    System.err.println("Failed to configure motor: " + status);
}
```

---

### 10. **Brake Mode Configuration Issues**
**File**: `Climber.java`

**Lines 17-20**:
```java
// TODO: STUDENTS - Ensure brake mode is ON so we don't fall!
// var cfg = new TalonFXConfiguration();
// cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
// m_climberMotor.getConfigurator().apply(cfg);
```

**Issue**: Critical safety feature (brake mode for climber) is commented out with a "TODO" for students.

**Impact**: Robot could fall during climb if students don't enable this.

**Recommendation**: Enable by default with comment explaining why it's important.

---

## 📋 RECOMMENDED ACTIONS

### Immediate (Pre-Student Use):
1. ✅ **Fix CAN ID mismatches** in either `Constants.java` OR subsystem files
2. ✅ **Fix method name calls** in `RobotContainer.java` (line 88, 89)
3. ✅ **Verify case of `generated` folder** path
4. ✅ **Add missing vendordeps** if using Epilogue logging
5. ✅ **Enable brake mode** on Climber by default

### Short-term:
6. ✅ **Align documentation** with actual code complexity
7. ✅ **Remove or document** CANrange dependency
8. ⚠️ **Decide**: Simple subsystems (as documented) OR advanced subsystems (as coded)?

### Long-term:
9. Add error handling to motor configurations
10. Create unit tests for subsystem initialization
11. Add logging/telemetry verification guide for students

---

## 🎯 CONCLUSION

The codebase has **two conflicting versions**:
1. **Documented Version**: Simple, student-friendly subsystems
2. **Actual Version**: Production-quality code with advanced features

**Next Steps**: 
- First engineer should decide which approach to use
- Update either code OR documentation to match
- Fix critical compilation errors before student deployment

**Estimated Fix Time**: 2-4 hours  
**Testing Required**: Full integration test with actual robot hardware
