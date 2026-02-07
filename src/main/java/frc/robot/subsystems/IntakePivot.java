package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake Pivot (TalonFX / Kraken).
 * - Position PID (degrees -> rotations)
 * - Velocity PID (deg/s or RPS helpers)
 * - Sim support with SingleJointedArmSim
 */
public class IntakePivot extends SubsystemBase {

  // =========================================================================
  // STUDENT ADJUSTMENT AREA: Intake Settings
  // =========================================================================

  // CAN / gearing
  private static final int CAN_ID = 15;
  /** Motor rotations per mechanism (pivot) rotation. */
  private static final double SENSOR_TO_MECH_RATIO = 80.0;

  // Slot 0 PID
  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Slot 0 FF (Phoenix 6 feedforward terms, volts-based)
  private static final double kS = 0.0;
  private static final double kV = 7.82;
  private static final double kA = 0.24;
  private static final double kG = 0.73;

  // Motion profile-ish limits (used by moveToAngleCommand)
  /** Max mechanism velocity (rad/s). */
  private static final double MAX_VELOCITY_RAD_PER_SEC = 1.0;
  /** Max mechanism acceleration (rad/s^2). */
  private static final double MAX_ACCEL_RAD_PER_SEC2 = 1.0;

  // Motor output behavior
  private static final boolean BRAKE_MODE = true;

  // Current limiting
  private static final boolean ENABLE_STATOR_LIMIT = true;
  private static final double STATOR_LIMIT_AMPS = 40.0;

  private static final boolean ENABLE_SUPPLY_LIMIT = false;
  private static final double SUPPLY_LIMIT_AMPS = 40.0;

  // Sim (24 inches -> meters)
  private static final double ARM_LENGTH_M = Units.inchesToMeters(24.0);

  // Pivot presets (degrees) - optional helpers
  private static final double STOW_DEG = 0.0;
  private static final double AMP_DEG = 90.0;

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  // Feedforward helper (WPILib ArmFeedforward expects radians-based values)
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  // Motor + requests
  private final TalonFX motor = new TalonFX(CAN_ID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Status signals
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();
  private final StatusSignal<Voltage> voltageSignal = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();
  private final StatusSignal<Temperature> temperatureSignal = motor.getDeviceTemp();

  // Simulation model (Falcon sim is “close enough” if you don’t have Kraken
  // helper in your WPILib)
  private final DCMotor dcMotor = DCMotor.getFalcon500(1);
  private final SingleJointedArmSim armSim;

  private double lastAngleDegSetpoint = STOW_DEG;

  public IntakePivot() {
    applyConfig();

    // Zero mechanism position at boot (only correct if you are physically at zero)
    motor.setPosition(0.0);

    armSim = new SingleJointedArmSim(
        dcMotor,
        SENSOR_TO_MECH_RATIO,
        SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, 2.26796),
        ARM_LENGTH_M,
        0.0,
        Math.PI / 2.0,
        true,
        0.0);
  }

  private void applyConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Slot 0 PID + FF
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kG = kG;

    // Current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = ENABLE_STATOR_LIMIT;
    currentLimits.StatorCurrentLimit = STATOR_LIMIT_AMPS;
    currentLimits.SupplyCurrentLimitEnable = ENABLE_SUPPLY_LIMIT;
    currentLimits.SupplyCurrentLimit = SUPPLY_LIMIT_AMPS;

    // Brake/coast
    config.MotorOutput.NeutralMode = BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Mechanism ratio (Phoenix: rotor -> mechanism)
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO;

    motor.getConfigurator().apply(config);
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
  }

  @Override
  public void simulationPeriodic() {
    // Feed TalonFX simulated motor voltage into the arm sim
    armSim.setInput(motor.getSimState().getMotorVoltage());

    // Update sim by 20ms
    armSim.update(0.020);

    // Battery sag
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Convert sim mechanism rad -> rotor rotations for CTRE sim
    double rotorPosRot = Radians.of(armSim.getAngleRads() * SENSOR_TO_MECH_RATIO).in(Rotations);
    double rotorVelRps = RadiansPerSecond.of(armSim.getVelocityRadPerSec() * SENSOR_TO_MECH_RATIO)
        .in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(rotorPosRot);
    motor.getSimState().setRotorVelocity(rotorVelRps);
  }

  // =========================================================================
  // TELEMETRY
  // =========================================================================

  /** Mechanism position in rotations (NOT rotor rotations). */
  @Logged(name = "IntakePivot/PositionRot")
  public double getPositionRotations() {
    return positionSignal.getValueAsDouble();
  }

  /** Mechanism angle in radians. */
  @Logged(name = "IntakePivot/PositionRad")
  public double getPositionRadians() {
    return getPositionRotations() * 2.0 * Math.PI;
  }

  /** Mechanism angle in degrees. */
  @Logged(name = "IntakePivot/PositionDeg")
  public double getPositionDegrees() {
    return getPositionRotations() * 360.0;
  }

  /** Mechanism velocity in rotations/sec. */
  @Logged(name = "IntakePivot/VelocityRPS")
  public double getVelocityRps() {
    return velocitySignal.getValueAsDouble();
  }

  /** Mechanism velocity in deg/sec. */
  @Logged(name = "IntakePivot/VelocityDegPerSec")
  public double getVelocityDegPerSec() {
    return getVelocityRps() * 360.0;
  }

  @Logged(name = "IntakePivot/AppliedVolts")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  @Logged(name = "IntakePivot/StatorCurrentA")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  @Logged(name = "IntakePivot/TempC")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  @Logged(name = "IntakePivot/AngleSetpointDeg")
  public double getLastAngleDegSetpoint() {
    return lastAngleDegSetpoint;
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** Position PID: set pivot angle in degrees. */
  public void setAngleDegrees(double angleDeg) {
    setAngleDegrees(angleDeg, 0.0);
  }

  /**
   * Position PID: set pivot angle in degrees, with optional accel term for FF.
   * accelRadPerSec2 is MECHANISM acceleration (rad/s^2).
   */
  public void setAngleDegrees(double angleDeg, double accelRadPerSec2) {
    lastAngleDegSetpoint = angleDeg;

    double angleRad = Units.degreesToRadians(angleDeg);
    double posRot = angleRad / (2.0 * Math.PI);

    // FF wants radians-based values (angle, velocity, accel)
    double velRadPerSec = getVelocityRps() * 2.0 * Math.PI;
    double ffVolts = feedforward.calculate(angleRad, velRadPerSec, accelRadPerSec2);

    motor.setControl(positionRequest.withPosition(posRot).withFeedForward(ffVolts));
  }

  /** Velocity PID: set pivot velocity in degrees/sec. */
  public void setVelocityDegPerSec(double velocityDegPerSec) {
    setVelocityDegPerSec(velocityDegPerSec, 0.0);
  }

  /**
   * Velocity PID: set pivot velocity in degrees/sec with optional accel (deg/s^2)
   * for FF.
   */
  public void setVelocityDegPerSec(double velocityDegPerSec, double accelDegPerSec2) {
    double velRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velRps = velRadPerSec / (2.0 * Math.PI);

    double accelRadPerSec2 = Units.degreesToRadians(accelDegPerSec2);

    double angleRad = getPositionRadians();
    double ffVolts = feedforward.calculate(angleRad, velRadPerSec, accelRadPerSec2);

    motor.setControl(velocityRequest.withVelocity(velRps).withFeedForward(ffVolts));
  }

  /** Velocity PID: set pivot velocity in MECHANISM rotations/sec (RPS). */
  public void setVelocityRps(double rps) {
    // Optional: clamp if you want
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  /** Direct voltage (bypasses PID). */
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public SingleJointedArmSim getSimulation() {
    return armSim;
  }

  // =========================================================================
  // COMMANDS (matches the style your RobotContainer expects)
  // =========================================================================

  /** Command: go to an angle (degrees). */
  public Command setAngleCommand(double angleDeg) {
    return runOnce(() -> setAngleDegrees(angleDeg));
  }

  /** Command: stop (zero velocity). */
  public Command stopCommand() {
    return runOnce(() -> setVelocityDegPerSec(0.0));
  }

  /**
   * Command: run at a given pivot velocity in RPS (used by your RobotContainer
   * POV testing).
   */
  public Command pivotRpsCommand(double rps) {
    return run(() -> setVelocityRps(rps)).finallyDo(interrupted -> setVelocityRps(0.0));
  }

  /** Command: run at a given velocity in deg/sec. */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocityDegPerSec(velocityDegPerSec))
        .finallyDo(interrupted -> setVelocityDegPerSec(0.0));
  }

  /**
   * Command: simple “drive to angle” using a crude proportional velocity (uses
   * MAX_VELOCITY_RAD_PER_SEC cap).
   */
  public Command moveToAngleCommand(double targetDeg) {
    return run(() -> {
      double currentDeg = getPositionDegrees();
      double errorDeg = targetDeg - currentDeg;

      double maxVelDegPerSec = Units.radiansToDegrees(MAX_VELOCITY_RAD_PER_SEC);
      double cmdVelDegPerSec = Math.signum(errorDeg) * Math.min(Math.abs(errorDeg) * 2.0, maxVelDegPerSec);

      setVelocityDegPerSec(cmdVelDegPerSec);
    })
        .until(() -> Math.abs(targetDeg - getPositionDegrees()) < 2.0)
        .finallyDo(interrupted -> setVelocityDegPerSec(0.0));
  }

  // Optional preset commands to match your earlier RobotContainer usage
  public Command pivotStow() {
    return setAngleCommand(STOW_DEG);
  }

  public Command pivotToAmp() {
    return setAngleCommand(AMP_DEG);
  }

  /**
   * Test pivot motor at specific velocity (RPS) for hardware validation.
   * Use this to verify motor wiring, direction, and encoder.
   */
  public Command testMotorCommand(double rps) {
    return run(() -> setVelocityRps(rps)).finallyDo(interrupted -> setVelocityRps(0.0));
  }

  // =========================================================================
  // UTIL
  // =========================================================================

  @SuppressWarnings("unused")
  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
