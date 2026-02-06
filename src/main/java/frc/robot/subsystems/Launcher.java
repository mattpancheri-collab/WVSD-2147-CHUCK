package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Launcher subsystem:
 * - Shooter: 3x Kraken X60 total (TalonFX)
 *   - ONE leader runs Velocity PID
 *   - Two followers follow the leader (alignment selectable)
 *   - 1:1 to two 4" flywheels
 * - Hood: 1x Kraken X44 (TalonFX) with POSITION PID
 *   - Hood gear ratio: 36.57 (motor rotations / hood rotations)
 *
 * Units:
 * - Shooter setpoints are in MECHANISM rotations/sec (RPS) after SensorToMechanismRatio.
 * - Hood setpoints are in DEGREES (converted internally to hood rotations).
 */
public class Launcher extends SubsystemBase {

  // =========================================================================
  // STUDENT ADJUSTMENT AREA: Launcher Settings
  // =========================================================================

  // CAN bus
  private static final String CAN_BUS = "rio"; // or "canivore"

  // Shooter CAN IDs (Kraken X60)
  private static final int LEADER_CAN_ID = 9;    // velocity PID leader
  private static final int FOLLOWER1_CAN_ID = 10; // follower
  private static final int FOLLOWER2_CAN_ID = 11; // follower

  // Shooter inversion (defines "positive" for the whole launcher)
  private static final InvertedValue LEADER_INVERTED = InvertedValue.CounterClockwise_Positive;

  /**
   * Followers alignment relative to the leader output.
   * - Aligned  = follow leader direction
   * - Opposed  = opposite direction (common for rollers facing each other)
   */
  private static final MotorAlignmentValue FOLLOWER1_ALIGNMENT = MotorAlignmentValue.Opposed; // TODO set
  private static final MotorAlignmentValue FOLLOWER2_ALIGNMENT = MotorAlignmentValue.Aligned; // TODO set

  private static final boolean SHOOTER_BRAKE_MODE = false; // shooter usually coasts

  // Shooter gear ratio (motor rotations / wheel rotations). 1:1 => 1.0
  private static final double SHOOTER_SENSOR_TO_MECH_RATIO = 1.0;

  // Shooter Velocity PID (Slot 0) on LEADER only
  private static final double SHOOTER_kP = 0.10; // TODO tune
  private static final double SHOOTER_kI = 0.0;
  private static final double SHOOTER_kD = 0.0;

  // Optional shooter FF (volts) - can stay 0 initially
  private static final double SHOOTER_kS = 0.0;
  private static final double SHOOTER_kV = 0.0;
  private static final double SHOOTER_kA = 0.0;

  // Shooter current limiting
  private static final boolean SHOOTER_ENABLE_STATOR_LIMIT = true;
  private static final double SHOOTER_STATOR_LIMIT_AMPS = 80.0;

  // Shooter RPS limits (Kraken X60 free ~90 RPS @ 12V)
  private static final double SHOOTER_MAX_RPS = 85.0;
  private static final double SHOOTER_RAMP_RPS_PER_SEC = 250.0;

  // Shooter presets (RPS)
  private static final double SHOOTER_IDLE_RPS = 10.0;
  private static final double SHOOTER_CLOSE_RPS = 45.0;
  private static final double SHOOTER_FAR_RPS = 65.0;

  // Hood CAN ID (Kraken X44)
  private static final int HOOD_CAN_ID = 12; // TODO set

  // Hood motor behavior
  private static final InvertedValue HOOD_INVERTED = InvertedValue.CounterClockwise_Positive;
  private static final boolean HOOD_BRAKE_MODE = true; // hold angle when stopped

  // Hood gear ratio (motor rotations / hood rotations) = 36.57:1 (motor:hood)
  private static final double HOOD_SENSOR_TO_MECH_RATIO = 36.57;

  // Hood position PID (Slot 0)
  private static final double HOOD_kP = 60.0; // TODO tune
  private static final double HOOD_kI = 0.0;
  private static final double HOOD_kD = 0.0;

  // Optional hood gravity/feedforward
  private static final GravityTypeValue HOOD_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
  private static final double HOOD_kS = 0.0;
  private static final double HOOD_kG = 0.0; // TODO set if hood droops
  private static final double HOOD_kV = 0.0;
  private static final double HOOD_kA = 0.0;

  // Hood limits (degrees)
  private static final double HOOD_MIN_DEG = 0.0;   // TODO set
  private static final double HOOD_MAX_DEG = 60.0;  // TODO set

  // Hood current limiting
  private static final boolean HOOD_ENABLE_STATOR_LIMIT = true;
  private static final double HOOD_STATOR_LIMIT_AMPS = 40.0;

  // Hood presets (degrees)
  private static final double HOOD_CLOSE_DEG = 20.0;
  private static final double HOOD_FAR_DEG = 40.0;

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  // Shooter motors
  private final TalonFX shooterLeader = new TalonFX(LEADER_CAN_ID, CAN_BUS);
  private final TalonFX shooterFollower1 = new TalonFX(FOLLOWER1_CAN_ID, CAN_BUS);
  private final TalonFX shooterFollower2 = new TalonFX(FOLLOWER2_CAN_ID, CAN_BUS);

  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0);

  // ✅ FIX: Your Phoenix version expects MotorAlignmentValue (not boolean, not TalonFX object)
  private final Follower follower1Request = new Follower(LEADER_CAN_ID, FOLLOWER1_ALIGNMENT);
  private final Follower follower2Request = new Follower(LEADER_CAN_ID, FOLLOWER2_ALIGNMENT);

  private final SlewRateLimiter shooterSetpointLimiter =
      new SlewRateLimiter(SHOOTER_RAMP_RPS_PER_SEC);

  private double shooterTargetRps = 0.0;

  // Hood motor (position PID)
  private final TalonFX hoodMotor = new TalonFX(HOOD_CAN_ID, CAN_BUS);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withSlot(0);

  private double hoodTargetDeg = HOOD_MIN_DEG;

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================

  public Launcher() {
    configureShooterLeader();
    configureShooterFollower(shooterFollower1);
    configureShooterFollower(shooterFollower2);

    // Put followers into follow mode (and we reassert in periodic for safety)
    shooterFollower1.setControl(follower1Request);
    shooterFollower2.setControl(follower2Request);

    configureHood();

    stop();
    setHoodDeg(hoodTargetDeg);
  }

  // -------------------------
  // Shooter config
  // -------------------------
  private void configureShooterLeader() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = SHOOTER_SENSOR_TO_MECH_RATIO;

    cfg.MotorOutput.NeutralMode =
        SHOOTER_BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = LEADER_INVERTED;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = SHOOTER_kP;
    s0.kI = SHOOTER_kI;
    s0.kD = SHOOTER_kD;
    s0.kS = SHOOTER_kS;
    s0.kV = SHOOTER_kV;
    s0.kA = SHOOTER_kA;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.StatorCurrentLimitEnable = SHOOTER_ENABLE_STATOR_LIMIT;
    cl.StatorCurrentLimit = SHOOTER_STATOR_LIMIT_AMPS;

    shooterLeader.getConfigurator().apply(cfg);
  }

  private void configureShooterFollower(TalonFX motor) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = SHOOTER_SENSOR_TO_MECH_RATIO;
    cfg.MotorOutput.NeutralMode =
        SHOOTER_BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.StatorCurrentLimitEnable = SHOOTER_ENABLE_STATOR_LIMIT;
    cl.StatorCurrentLimit = SHOOTER_STATOR_LIMIT_AMPS;

    motor.getConfigurator().apply(cfg);
  }

  // -------------------------
  // Hood config (POSITION PID)
  // -------------------------
  private void configureHood() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = HOOD_SENSOR_TO_MECH_RATIO;

    cfg.MotorOutput.NeutralMode =
        HOOD_BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = HOOD_INVERTED;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = HOOD_kP;
    s0.kI = HOOD_kI;
    s0.kD = HOOD_kD;

    s0.GravityType = HOOD_GRAVITY_TYPE;
    s0.kS = HOOD_kS;
    s0.kG = HOOD_kG;
    s0.kV = HOOD_kV;
    s0.kA = HOOD_kA;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.StatorCurrentLimitEnable = HOOD_ENABLE_STATOR_LIMIT;
    cl.StatorCurrentLimit = HOOD_STATOR_LIMIT_AMPS;

    hoodMotor.getConfigurator().apply(cfg);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  public void setShooterRps(double rps) {
    shooterTargetRps = clamp(rps, 0.0, SHOOTER_MAX_RPS);
  }

  public void setHoodDeg(double deg) {
    hoodTargetDeg = clamp(deg, HOOD_MIN_DEG, HOOD_MAX_DEG);
  }

  public void stop() {
    setShooterRps(0.0);
    shooterLeader.stopMotor();
    shooterFollower1.stopMotor();
    shooterFollower2.stopMotor();
    // Hood keeps holding target.
  }

  public void idleShooter() {
    setShooterRps(SHOOTER_IDLE_RPS);
  }

  public void setCloseShot() {
    setShooterRps(SHOOTER_CLOSE_RPS);
    setHoodDeg(HOOD_CLOSE_DEG);
  }

  public void setFarShot() {
    setShooterRps(SHOOTER_FAR_RPS);
    setHoodDeg(HOOD_FAR_DEG);
  }

  // =========================================================================
  // TELEMETRY
  // =========================================================================

  @Logged(name = "Launcher/ShooterTargetRPS")
  public double getShooterTargetRps() {
    return shooterTargetRps;
  }

  @Logged(name = "Launcher/ShooterLeaderVelocityRPS")
  public double getShooterLeaderVelocityRps() {
    return shooterLeader.getVelocity().getValueAsDouble();
  }

  @Logged(name = "Launcher/HoodTargetDeg")
  public double getHoodTargetDeg() {
    return hoodTargetDeg;
  }

  @Logged(name = "Launcher/HoodPositionDeg")
  public double getHoodPositionDeg() {
    return hoodMotor.getPosition().getValueAsDouble() * 360.0; // hood rotations -> degrees
  }

  public boolean shooterAtSetpoint(double toleranceRps) {
    return Math.abs(getShooterLeaderVelocityRps() - shooterTargetRps) <= toleranceRps;
  }

  public boolean hoodAtSetpoint(double toleranceDeg) {
    return Math.abs(getHoodPositionDeg() - hoodTargetDeg) <= toleranceDeg;
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    // Keep followers latched
    shooterFollower1.setControl(follower1Request);
    shooterFollower2.setControl(follower2Request);

    // Shooter velocity control (leader only)
    double limitedRps = shooterSetpointLimiter.calculate(shooterTargetRps);
    shooterLeader.setControl(shooterLeaderRequest.withVelocity(limitedRps));

    // Hood position control: degrees -> hood rotations
    double hoodRotations = hoodTargetDeg / 360.0;
    hoodMotor.setControl(hoodPositionRequest.withPosition(hoodRotations));
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command closeShotCommand() {
    return runOnce(this::setCloseShot);
  }

  public Command farShotCommand() {
    return runOnce(this::setFarShot);
  }

  public Command setHoodDegCommand(double deg) {
    return runOnce(() -> setHoodDeg(deg));
  }

  public Command runShooterRpsCommand(double rps) {
    return run(() -> setShooterRps(rps)).finallyDo(i -> setShooterRps(0.0));
  }

  // =========================================================================
  // UTIL
  // =========================================================================

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
