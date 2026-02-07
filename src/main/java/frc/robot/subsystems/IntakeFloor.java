package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeFloorConstants;
import frc.robot.Constants.CANBus;

/**
 * Intake Floor subsystem:
 * - Controls the intake rollers.
 * - Simple velocity control for intaking and ejecting.
 */
public class IntakeFloor extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS
  // =========================================================================

  /** ✅ Intake Roller Motor (TalonFX) */
  private final TalonFX intakeMotor = new TalonFX(IntakeFloorConstants.kIntakeID, CANBus.kDefaultBus);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  /** 📉 Slew rate limiter for smooth motor response */
  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(IntakeFloorConstants.kRampRPSPerSec);

  private double targetRps = 0.0;

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================

  public IntakeFloor() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Feedback scaling
    config.Feedback.SensorToMechanismRatio = 1.0;

    // Output settings
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Slot 0 PID + FF
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = IntakeFloorConstants.kP;
    slot0.kI = IntakeFloorConstants.kI;
    slot0.kD = IntakeFloorConstants.kD;
    // slot0.kS = 0.0; // Removed
    // slot0.kV = 0.12; // Removed
    // slot0.kA = 0.0; // Removed

    // Current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = IntakeFloorConstants.kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = IntakeFloorConstants.kStatorLimitAmps;

    intakeMotor.getConfigurator().apply(config);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** 🏎️ Run the intake rollers "in". */
  public void intakeIn() {
    setRps(IntakeFloorConstants.kIntakeInRPS);
  }

  /** 🏎️ Run the intake rollers "out" (eject). */
  public void intakeOut() {
    setRps(IntakeFloorConstants.kIntakeOutRPS);
  }

  /** 🏎️ Set specific rotations per second. */
  public void setRps(double rps) {
    this.targetRps = clamp(rps, -IntakeFloorConstants.kMaxRPS, IntakeFloorConstants.kMaxRPS);
  }

  /** 👋 Stop the intake rollers. */
  public void stop() {
    targetRps = 0.0;
    intakeMotor.stopMotor();
  }

  // =========================================================================
  // TELEMETRY
  // =========================================================================

  @Logged(name = "IntakeFloor/TargetRPS")
  public double getTargetRps() {
    return targetRps;
  }

  @Logged(name = "IntakeFloor/VelocityRPS")
  public double getVelocityRps() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    // Slew limit the setpoint so the intake doesn't "snap" to speed and brown out.
    double limitedRps = rpsLimiter.calculate(targetRps);
    intakeMotor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  /** 🎯 Intake command. */
  public Command intakeInCommand() {
    return run(this::intakeIn).finallyDo(interrupted -> stop());
  }

  /** 🎯 Eject command. */
  public Command intakeOutCommand() {
    return run(this::intakeOut).finallyDo(interrupted -> stop());
  }

  public Command intakeCommand(double rps) {
    return run(() -> setRps(rps)).finallyDo(interrupted -> stop());
  }

  /**
   * Test intake motor at specific RPS for hardware validation.
   * Use this to verify motor wiring and direction.
   */
  public Command testMotorCommand(double rps) {
    return run(() -> setRps(rps)).finallyDo(interrupted -> stop());
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  // =========================================================================
  // UTIL
  // =========================================================================

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}