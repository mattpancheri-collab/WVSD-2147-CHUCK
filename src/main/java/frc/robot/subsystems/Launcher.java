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

import static frc.robot.Constants.LauncherConstants.*;
import static frc.robot.Constants.CANBus.kDefaultBus;

/**
 * Launcher subsystem:
 * - Shooter: 3x Kraken X60 total (TalonFX)
 * - ONE leader runs Velocity PID
 * - Two followers follow the leader (alignment selectable)
 * - 1:1 to two 4" flywheels
 * - Hood: 1x Kraken X44 (TalonFX) with POSITION PID
 * - Hood gear ratio: 36.57 (motor rotations / hood rotations)
 *
 * Units:
 * - Shooter setpoints are in MECHANISM rotations/sec (RPS).
 * - Hood setpoints are in DEGREES (converted internally to hood rotations).
 */
public class Launcher extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS
  // =========================================================================

  /** Shooter Leader (Kraken X60) - Handles Velocity PID control */
  private final TalonFX shooterLeader = new TalonFX(kShooterLeaderID, kDefaultBus);
  /** Shooter Follower 1 (Kraken X60) */
  private final TalonFX shooterFollower1 = new TalonFX(kShooterFollower1ID, kDefaultBus);
  /** Shooter Follower 2 (Kraken X60) */
  private final TalonFX shooterFollower2 = new TalonFX(kShooterFollower2ID, kDefaultBus);

  /** Hood Motor (Kraken X44) - Handles Position PID control */
  private final TalonFX hoodMotor = new TalonFX(kHoodID, kDefaultBus);

  // Control Requests
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withSlot(0);

  /**
   * Followers alignment relative to the leader output.
   * - Aligned = follow leader direction
   * - Opposed = opposite direction (common for rollers facing each other)
   */
  private final Follower follower1Request = new Follower(kShooterLeaderID, MotorAlignmentValue.Opposed);
  private final Follower follower2Request = new Follower(kShooterLeaderID, MotorAlignmentValue.Aligned);

  /** 📉 Slew rate limiter to smooth out shooter speed changes */
  private final SlewRateLimiter shooterSetpointLimiter = new SlewRateLimiter(kShooterRampRPSPerSec);

  private double shooterTargetRps = 0.0;
  private double hoodTargetDeg = kHoodMinDeg;

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
    setHoodDegrees(hoodTargetDeg);
  }

  // -------------------------
  // Shooter config
  // -------------------------
  private void configureShooterLeader() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // 1:1 ratio
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kShooterP;
    slot0.kI = kShooterI;
    slot0.kD = kShooterD;
    slot0.kS = kShooterS;
    slot0.kV = kShooterV;
    slot0.kA = kShooterA;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = kShooterEnableStatorLimit;
    currentLimits.StatorCurrentLimit = kShooterStatorLimitAmps;

    shooterLeader.getConfigurator().apply(config);
  }

  private void configureShooterFollower(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  // -------------------------
  // Hood config (POSITION PID)
  // -------------------------
  private void configureHood() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = kHoodGearRatio;

    cfg.MotorOutput.NeutralMode = kHoodEnableStatorLimit ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = kHoodP;
    s0.kI = kHoodI;
    s0.kD = kHoodD;

    s0.GravityType = GravityTypeValue.Arm_Cosine;
    s0.kS = kHoodS;
    s0.kG = kHoodG;
    s0.kV = kHoodV;
    s0.kA = kHoodA;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.StatorCurrentLimitEnable = kHoodEnableStatorLimit;
    cl.StatorCurrentLimit = kHoodStatorLimitAmps;

    hoodMotor.getConfigurator().apply(cfg);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /**
   * Set the shooter target speed.
   * 
   * @param rps Target rotations per second for the flywheels.
   */
  public void setShooterRps(double rps) {
    shooterTargetRps = clamp(rps, 0.0, kShooterMaxRPS);
  }

  /**
   * Set the hood target angle.
   * 
   * @param deg Target angle in degrees.
   */
  public void setHoodDegrees(double deg) {
    hoodTargetDeg = clamp(deg, kHoodMinDeg, kHoodMaxDeg);
  }

  /** Stops all motors in the launcher. */
  public void stop() {
    setShooterRps(0.0);
    shooterLeader.stopMotor();
    shooterFollower1.stopMotor();
    shooterFollower2.stopMotor();
  }

  /** Sets shooter to idle speed. */
  public void idleShooter() {
    setShooterRps(kShooterIdleRPS);
  }

  /** Sets up the launcher for a close-range shot. */
  public void setCloseShot() {
    setShooterRps(kShooterCloseRPS);
    setHoodDegrees(kHoodCloseDeg);
  }

  /** Sets up the launcher for a far-range shot. */
  public void setFarShot() {
    setShooterRps(kShooterFarRPS);
    setHoodDegrees(kHoodFarDeg);
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

  /**
   * Command to stop all launcher motors.
   * 
   * @return A command that stops the launcher.
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /**
   * Command to set the launcher for a close shot.
   * 
   * @return A command that configures the launcher for a close shot.
   */
  public Command closeShotCommand() {
    return runOnce(this::setCloseShot);
  }

  /**
   * Command to set the launcher for a far shot.
   * 
   * @return A command that configures the launcher for a far shot.
   */
  public Command farShotCommand() {
    return runOnce(this::setFarShot);
  }

  /**
   * Command to set the hood to a specific angle.
   * 
   * @param deg The target angle in degrees.
   * @return A command that sets the hood angle.
   */
  public Command setHoodDegreesCommand(double deg) {
    return runOnce(() -> setHoodDegrees(deg));
  }

  /**
   * Command to run the shooter at a specific RPS, then stop it.
   * 
   * @param rps The target rotations per second for the shooter.
   * @return A command that runs the shooter and then stops it.
   */
  public Command runShooterRpsCommand(double rps) {
    return run(() -> setShooterRps(rps)).finallyDo(i -> setShooterRps(0.0));
  }

  // =========================================================================
  // INDIVIDUAL MOTOR TESTING (for hardware validation)
  // =========================================================================

  /**
   * Test shooter LEADER motor only at a specific RPS.
   * Followers will NOT move in this mode.
   * Use for hardware/wiring validation.
   * 
   * @param rps The target rotations per second for the leader motor.
   * @return A command to test the shooter leader.
   */
  public Command testShooterLeaderCommand(double rps) {
    return run(() -> {
      shooterLeader.setControl(shooterLeaderRequest.withVelocity(rps));
      shooterFollower1.stopMotor();
      shooterFollower2.stopMotor();
    }).finallyDo(interrupted -> {
      shooterLeader.stopMotor();
    });
  }

  /**
   * Test shooter FOLLOWER 1 motor only at a specific RPS.
   * Leader and Follower 2 will NOT move.
   * Use for hardware/wiring validation.
   * 
   * @param rps The target rotations per second for follower 1.
   * @return A command to test shooter follower 1.
   */
  public Command testShooterFollower1Command(double rps) {
    return run(() -> {
      shooterLeader.stopMotor();
      shooterFollower1.setControl(new VelocityVoltage(rps).withSlot(0));
      shooterFollower2.stopMotor();
    }).finallyDo(interrupted -> {
      shooterFollower1.stopMotor();
    });
  }

  /**
   * Test shooter FOLLOWER 2 motor only at a specific RPS.
   * Leader and Follower 1 will NOT move.
   * Use for hardware/wiring validation.
   * 
   * @param rps The target rotations per second for follower 2.
   * @return A command to test shooter follower 2.
   */
  public Command testShooterFollower2Command(double rps) {
    return run(() -> {
      shooterLeader.stopMotor();
      shooterFollower1.stopMotor();
      shooterFollower2.setControl(new VelocityVoltage(rps).withSlot(0));
    }).finallyDo(interrupted -> {
      shooterFollower2.stopMotor();
    });
  }

  /**
   * Test hood motor at a specific target angle in degrees.
   * Shooter motors will NOT move.
   * Use for hardware/wiring validation.
   * 
   * @param targetDeg The target angle in degrees for the hood.
   * @return A command to test the hood motor.
   */
  public Command testHoodCommand(double targetDeg) {
    return run(() -> {
      shooterLeader.stopMotor();
      shooterFollower1.stopMotor();
      shooterFollower2.stopMotor();
      double hoodRot = clamp(targetDeg, kHoodMinDeg, kHoodMaxDeg) / 360.0;
      hoodMotor.setControl(hoodPositionRequest.withPosition(hoodRot));
    });
  }

  // =========================================================================
  // UTIL
  // =========================================================================

  /**
   * Clamps a value between a minimum and maximum.
   * 
   * @param val The value to clamp.
   * @param min The minimum allowed value.
   * @param max The maximum allowed value.
   * @return The clamped value.
   */
  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
