package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

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
import frc.robot.Constants.BusConstants;
import frc.robot.Constants.CANConstants;

/**
 * Intake Pivot subsystem:
 * - POSITION control for angles (0 deg = intake, 90 deg = idle).
 * - Voltage override mode for testing controller (open-loop volts).
 *
 * Notes:
 * - Angle setpoints are in DEGREES.
 * - Internally converted to motor rotations using kGearRatio.
 */
public class IntakePivot extends SubsystemBase {

  // =========================================================================
  // CONTROL MODE
  // =========================================================================

  /**
   * Tracks which control mode the pivot is currently in.
   * Only ONE mode is active at a time — periodic() dispatches on this
   * to ensure a single, unambiguous command reaches the motor each loop.
   */
  private enum ControlMode {
    /**
     * PID position control — setpoint already sent to motor in setAngleDegrees().
     */
    POSITION,
    /** Closed-loop velocity control — periodic() applies the slew-limited RPS. */
    VELOCITY,
    /** Open-loop voltage (testing only) — periodic() applies voltageDemand. */
    VOLTAGE,
    /** Motor output is zero. */
    STOPPED
  }

  // =========================================================================
  // HARDWARE / INTERNALS
  // =========================================================================

  private final TalonFX pivotMotor = new TalonFX(CANConstants.kPivotID, BusConstants.kDefaultBus);

  // Requests
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(IntakePivotConstants.kRampRPSPerSec);

  // =========================================================================
  // STATE
  // =========================================================================

  private ControlMode m_controlMode = ControlMode.STOPPED;

  private double targetRps = 0.0;
  private double targetDeg = IntakePivotConstants.kIdleAngleDeg;
  private double voltageDemand = 0.0;

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================

  public IntakePivot() {
    configureMotor();
    // start at idle (90 deg)
    setAngleDegrees(IntakePivotConstants.kIdleAngleDeg);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // IMPORTANT: Use your real gear ratio (motor rotations per pivot rotation)
    config.Feedback.SensorToMechanismRatio = IntakePivotConstants.kGearRatio;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Slot 0 PID for POSITION control
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = IntakePivotConstants.kP;
    slot0.kI = IntakePivotConstants.kI;
    slot0.kD = IntakePivotConstants.kD;
    slot0.kG = IntakePivotConstants.kG;
    slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;

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
    targetRps = 0.0;
    rpsLimiter.reset(0.0);
    voltageDemand = 0.0;

    targetDeg = MathUtil.clamp(deg, 0.0, 90.0);

    // Phoenix PositionVoltage uses "mechanism rotations" because we set
    // SensorToMechanismRatio.
    // Mechanism rotations = degrees / 360
    double pivotRotations = targetDeg / 360.0;
    pivotMotor.setControl(positionRequest.withPosition(pivotRotations));

    m_controlMode = ControlMode.POSITION;
  }

  /**
   * Velocity mode (closed-loop RPS). periodic() applies the slew-limited
   * setpoint.
   */
  public void setRps(double rps) {
    voltageDemand = 0.0;

    final double maxRps = IntakePivotConstants.kMaxVelocityRPS;
    targetRps = MathUtil.clamp(rps, -maxRps, maxRps);

    m_controlMode = ControlMode.VELOCITY;
  }

  /**
   * Testing mode: open-loop volts. periodic() applies voltageDemand each loop.
   */
  public void setVoltage(double volts) {
    targetRps = 0.0;
    rpsLimiter.reset(0.0);

    voltageDemand = MathUtil.clamp(volts, -12.0, 12.0);

    m_controlMode = ControlMode.VOLTAGE;
  }

  /** Stop all pivot output. */
  public void stop() {
    targetRps = 0.0;
    rpsLimiter.reset(0.0);
    voltageDemand = 0.0;

    pivotMotor.setVoltage(0.0);
    m_controlMode = ControlMode.STOPPED;
  }

  // Convenience helpers
  public void deploy() {
    setAngleDegrees(IntakePivotConstants.kIntakeAngleDeg);
  }

  public void stow() {
    setAngleDegrees(IntakePivotConstants.kIdleAngleDeg);
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
    return m_controlMode == ControlMode.VOLTAGE;
  }

  public double getVoltageDemand() {
    return voltageDemand;
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case POSITION:
        // Re-apply position request in every loop to ensure robustness.
        double pivotRotations = targetDeg / 360.0;
        pivotMotor.setControl(positionRequest.withPosition(pivotRotations));
        break;

      case VELOCITY:
        double limitedRps = rpsLimiter.calculate(targetRps);
        pivotMotor.setControl(velocityRequest.withVelocity(limitedRps));
        break;

      case VOLTAGE:
        pivotMotor.setVoltage(voltageDemand);
        break;

      case STOPPED:
      default:
        // Motor output is already zero (set in stop()).
        break;
    }
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  /** While held, deploy to 90 degrees. On release, stow to 0 degrees. */
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
        this);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
