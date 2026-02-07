package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.CANBus;

/**
 * Intake Pivot subsystem:
 * - Controls the angle of the intake using a Kraken motor.
 * - Uses Position PID to move between Stowed, Deployed, and Amp positions.
 * - Implements gravity compensation (kG) to prevent the intake from falling.
 */
public class IntakePivot extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS
  // =========================================================================

  /** Pivot Motor (Kraken X60) - Handles Position PID control */
  private final TalonFX pivotMotor = new TalonFX(IntakePivotConstants.kPivotID, CANBus.kDefaultBus);

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double targetAngleDeg = IntakePivotConstants.kStowAngleDeg;

  public IntakePivot() {
    configureMotor();
    setAngleDeg(IntakePivotConstants.kStowAngleDeg);
  }

  private void configureMotor() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = IntakePivotConstants.kGearRatio;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = IntakePivotConstants.kP;
    s0.kI = IntakePivotConstants.kI;
    s0.kD = IntakePivotConstants.kD;
    s0.kG = IntakePivotConstants.kG;
    s0.kV = IntakePivotConstants.kV;
    s0.kA = IntakePivotConstants.kA;
    s0.GravityType = GravityTypeValue.Arm_Cosine;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.StatorCurrentLimitEnable = IntakePivotConstants.kEnableStatorLimit;
    cl.StatorCurrentLimit = IntakePivotConstants.kStatorLimitAmps;

    pivotMotor.getConfigurator().apply(cfg);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** Helper to set a specific angle in degrees. */
  public void setAngleDeg(double deg) {
    targetAngleDeg = clamp(deg, IntakePivotConstants.kMinAngleDeg, IntakePivotConstants.kMaxAngleDeg);
  }

  /** Stop moving. */
  public void stop() {
    pivotMotor.stopMotor();
  }

  /** Move to the stowed position. */
  public void stow() {
    setAngleDeg(IntakePivotConstants.kStowAngleDeg);
  }

  /** Deploy the intake for floor pick-up. */
  public void deploy() {
    setAngleDeg(IntakePivotConstants.kDeployAngleDeg);
  }

  /** Set angle for Amp scoring. */
  public void ampPosition() {
    setAngleDeg(IntakePivotConstants.kAmpAngleDeg);
  }

  // =========================================================================
  // TELEMETRY
  // =========================================================================

  @Logged(name = "IntakePivot/TargetAngleDeg")
  public double getTargetAngleDeg() {
    return targetAngleDeg;
  }

  @Logged(name = "IntakePivot/CurrentAngleDeg")
  public double getAngleDeg() {
    return pivotMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public boolean atSetpoint(double toleranceDeg) {
    return Math.abs(getAngleDeg() - targetAngleDeg) <= toleranceDeg;
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    // Current degrees -> mechanism rotations
    double rotations = targetAngleDeg / 360.0;
    pivotMotor.setControl(positionRequest.withPosition(rotations));
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  /** 🎯 Command to stow the intake. */
  public Command pivotStowCommand() {
    return runOnce(this::stow);
  }

  /** 🎯 Command to deploy the intake. */
  public Command deployCommand() {
    return runOnce(this::deploy);
  }

  /** 🎯 Command to move to Amp position. */
  public Command pivotToAmpCommand() {
    return runOnce(this::ampPosition);
  }

  /** 🎯 Command to set a specific angle. */
  public Command setAngleCommand(double deg) {
    return runOnce(() -> setAngleDeg(deg));
  }

  /**
   * 🎯 Command for manual velocity control (rotations per second). Stops on
   * finish.
   */
  public Command pivotRpsCommand(double rps) {
    return run(() -> {
      double clampedRps = clamp(rps, -IntakePivotConstants.kMaxVelocityRPS, IntakePivotConstants.kMaxVelocityRPS);
      pivotMotor.setControl(velocityRequest.withVelocity(clampedRps));
    }).finallyDo(interrupted -> stop());
  }

  /** 👋 Command to stop the pivot. */
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
