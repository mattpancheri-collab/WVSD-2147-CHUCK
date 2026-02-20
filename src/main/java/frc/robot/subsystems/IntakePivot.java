package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.CANBus;

/**
 * Intake Pivot subsystem:
 * - POSITION control for angles (0 deg = stow, 90 deg = deploy).
 * - Voltage override mode for testing controller (open-loop volts).
 *
 * Notes:
 * - Angle setpoints are in DEGREES.
 * - Internally converted to motor rotations using kGearRatio.
 */
public class IntakePivot extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS
  // =========================================================================

  private final TalonFX pivotMotor =
      new TalonFX(IntakePivotConstants.kPivotID, CANBus.kDefaultBus);

  // Requests
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0); // optional/manual

  // Optional slew limiter if you still use velocity mode anywhere
  private final SlewRateLimiter rpsLimiter =
      new SlewRateLimiter(IntakePivotConstants.kRampRPSPerSec);

  // =========================================================================
  // STATE
  // =========================================================================

  private double targetRps = 0.0;     // only used for optional velocity mode
  private double targetDeg = IntakePivotConstants.kMinAngleDeg;

  // Voltage override (testing)
  private boolean voltageOverride = false;
  private double voltageDemand = 0.0;

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================

  public IntakePivot() {
    configureMotor();
    // start stowed
    setAngleDegrees(IntakePivotConstants.kMinAngleDeg);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // IMPORTANT: Use your real gear ratio (motor rotations per pivot rotation)
    config.Feedback.SensorToMechanismRatio = IntakePivotConstants.kGearRatio;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // flip if needed

    // Slot 0 PID for POSITION control
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = IntakePivotConstants.kP;
    slot0.kI = IntakePivotConstants.kI;
    slot0.kD = IntakePivotConstants.kD;

    // Current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = IntakePivotConstants.kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = IntakePivotConstants.kStatorLimitAmps;

    pivotMotor.getConfigurator().apply(config);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** Position mode: move pivot to an angle in degrees and hold it there. */
  public void setAngleDegrees(double deg) {
    // Exit voltage override when using PID position
    voltageOverride = false;
    voltageDemand = 0.0;

    // Stop any velocity intent
    targetRps = 0.0;
    rpsLimiter.reset(0.0);

    targetDeg = clamp(deg, IntakePivotConstants.kMinAngleDeg, IntakePivotConstants.kMaxAngleDeg);

    // Phoenix PositionVoltage uses "mechanism rotations" because we set SensorToMechanismRatio.
    // Mechanism rotations = degrees / 360
    double pivotRotations = targetDeg / 360.0;
    pivotMotor.setControl(positionRequest.withPosition(pivotRotations));
  }

  /** Optional: velocity mode (manual). Exits voltage override. */
  public void setRps(double rps) {
    voltageOverride = false;
    voltageDemand = 0.0;

    // FIX: kMaxVelocityRPS is already a double (RPS), so don't call .in(...)
    final double maxRps = IntakePivotConstants.kMaxVelocityRPS;
    targetRps = clamp(rps, -maxRps, maxRps);
  }

  /** Testing mode: open-loop volts. */
  public void setVoltage(double volts) {
    voltageOverride = true;
    voltageDemand = clamp(volts, -12.0, 12.0);

    // Clear other intents so nothing “wakes up” later
    targetRps = 0.0;
    rpsLimiter.reset(0.0);
  }

  /** Stop pivot output. (Does NOT change the target angle.) */
  public void stop() {
    voltageOverride = false;
    voltageDemand = 0.0;

    targetRps = 0.0;
    rpsLimiter.reset(0.0);

    pivotMotor.setVoltage(0.0);
  }

  // Convenience helpers
  public void deploy() {
    setAngleDegrees(IntakePivotConstants.kDeployAngleDeg);
  }

  public void stow() {
    setAngleDegrees(IntakePivotConstants.kMinAngleDeg);
  }

  // =========================================================================
  // ACCESSORS
  // =========================================================================

  public double getTargetDeg() {
    return targetDeg;
  }

  public double getPositionDeg() {
    return pivotMotor.getPosition().getValueAsDouble() * 360.0; // mechanism rotations -> degrees
  }

  public boolean isVoltageOverride() {
    return voltageOverride;
  }

  public double getVoltageDemand() {
    return voltageDemand;
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    // Voltage override wins
    if (voltageOverride) {
      pivotMotor.setVoltage(voltageDemand);
      return;
    }

    // If you're using velocity mode anywhere, keep it alive here.
    // If you never use setRps(), you can delete this block.
    if (Math.abs(targetRps) > 1e-6) {
      double limitedRps = rpsLimiter.calculate(targetRps);
      pivotMotor.setControl(velocityRequest.withVelocity(limitedRps));
    }
    // Position mode is commanded directly in setAngleDegrees()
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  /** While held, deploy to 90°. On release, stow to 0°. */
  public Command deployWhileHeldStowOnReleaseCommand() {
    return Commands.startEnd(this::deploy, this::stow, this);
  }

  public Command pivotDeployCommand() {
    return run(this::deploy);
  }

  public Command pivotStowCommand() {
    return run(this::stow);
  }

  public Command setAngleCommand(double deg) {
    return run(() -> setAngleDegrees(deg));
  }

  public Command pivotVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> setVoltage(volts),
        () -> setVoltage(0.0),
        this
    );
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
