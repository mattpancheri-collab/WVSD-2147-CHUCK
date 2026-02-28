package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.BusConstants.kDefaultBus;
import static frc.robot.Constants.LauncherConstants.*;
import static frc.robot.Constants.ShootingConstants.kShooterPolarity;
import frc.robot.Constants.CANConstants;

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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  // Shooter motors
  private final TalonFX shooterLeader = new TalonFX(CANConstants.kShooterLeaderID, kDefaultBus);
  private final TalonFX shooterFollower1 = new TalonFX(CANConstants.kShooterFollower1ID, kDefaultBus);
  private final TalonFX shooterFollower2 = new TalonFX(CANConstants.kShooterFollower2ID, kDefaultBus);

  // Hood motor
  private final TalonFX hoodMotor = new TalonFX(CANConstants.kHoodID, kDefaultBus);

  private enum ControlMode {
    VELOCITY,
    VOLTAGE,
    STOPPED
  }

  private ControlMode m_controlMode = ControlMode.STOPPED;

  // Requests
  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withSlot(0);

  private final Follower follower1Request = new Follower(CANConstants.kShooterLeaderID, MotorAlignmentValue.Aligned);

  private final Follower follower2Request = new Follower(CANConstants.kShooterLeaderID, MotorAlignmentValue.Opposed);

  private final SlewRateLimiter shooterSetpointLimiter = new SlewRateLimiter(kShooterRampRPSPerSec);

  // State targets
  private double shooterTargetRps = 0.0;
  private double hoodTargetDeg = kHoodMinDeg;
  private double shooterVoltageDemand = 0.0;

  public Launcher() {
    configureShooterLeader();
    configureShooterFollower(shooterFollower1);
    configureShooterFollower(shooterFollower2);
    configureHood();

    // latch followers once (also re-latched in periodic)
    shooterFollower1.setControl(follower1Request);
    shooterFollower2.setControl(follower2Request);

    stop();
    setHoodDegrees(hoodTargetDeg);
  }

  // ---------------------------------------------------------------------------
  // Config
  // ---------------------------------------------------------------------------

  private void configureShooterLeader() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = 1.0;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // You verified in Phoenix Tuner that CW+ with +voltage is "correct".
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = kShooterP;
    s0.kI = kShooterI;
    s0.kD = kShooterD;
    s0.kS = kShooterS;
    s0.kV = kShooterV;
    s0.kA = kShooterA;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.StatorCurrentLimitEnable = kShooterEnableStatorLimit;
    cl.StatorCurrentLimit = kShooterStatorLimitAmps;

    shooterLeader.getConfigurator().apply(cfg);
  }

  private void configureShooterFollower(TalonFX motor) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(cfg);
  }

  private void configureHood() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = kHoodGearRatio;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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

  // ---------------------------------------------------------------------------
  // Control API
  // ---------------------------------------------------------------------------

  /** Velocity mode (exits voltage override). */
  public void setShooterRps(double rps) {
    shooterVoltageDemand = 0.0;

    // Apply polarity here so ALL setpoints match the same "forward"
    double signedRps = kShooterPolarity * rps;

    // Clamp to [0..max] because your shooter API expects "positive is shoot"
    shooterTargetRps = MathUtil.clamp(signedRps, 0.0, kShooterMaxRPS);

    m_controlMode = ControlMode.VELOCITY;
  }

  /** Testing mode: open-loop volts on shooter (leader + both followers). */
  public void setVoltage(double volts) {
    shooterTargetRps = 0.0;
    shooterSetpointLimiter.reset(0.0);

    // Apply polarity once, globally
    shooterVoltageDemand = MathUtil.clamp(kShooterPolarity * volts, -12.0, 12.0);

    m_controlMode = ControlMode.VOLTAGE;
  }

  public void setHoodDegrees(double deg) {
    hoodTargetDeg = MathUtil.clamp(deg, kHoodMinDeg, kHoodMaxDeg);
  }

  public void stop() {
    shooterVoltageDemand = 0.0;
    shooterTargetRps = 0.0;
    shooterSetpointLimiter.reset(0.0);

    shooterLeader.setVoltage(0.0);
    shooterFollower1.setVoltage(0.0);
    shooterFollower2.setVoltage(0.0);

    m_controlMode = ControlMode.STOPPED;
  }

  public void idleShooter() {
    setShooterRps(kShooterIdleRPS);
  }

  public void setCloseShot() {
    setShooterRps(kShooterCloseRPS);
    setHoodDegrees(kHoodCloseDeg);
  }

  public void setFarShot() {
    setShooterRps(kShooterFarRPS);
    setHoodDegrees(kHoodFarDeg);
  }

  // ---------------------------------------------------------------------------
  // Accessors
  // ---------------------------------------------------------------------------

  public double getShooterTargetRps() {
    return shooterTargetRps;
  }

  public double getShooterLeaderVelocityRps() {
    return shooterLeader.getVelocity().getValueAsDouble();
  }

  public double getHoodTargetDeg() {
    return hoodTargetDeg;
  }

  public double getHoodPositionDeg() {
    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // keep followers latched
    shooterFollower1.setControl(follower1Request);
    shooterFollower2.setControl(follower2Request);

    switch (m_controlMode) {
      case VELOCITY:
        double limitedRps = shooterSetpointLimiter.calculate(shooterTargetRps);
        shooterLeader.setControl(shooterLeaderRequest.withVelocity(limitedRps));
        break;

      case VOLTAGE:
        shooterLeader.setVoltage(shooterVoltageDemand);
        shooterFollower1.setVoltage(shooterVoltageDemand);
        shooterFollower2.setVoltage(shooterVoltageDemand);
        break;

      case STOPPED:
      default:
        // Already handled by stop() calls or startup
        break;
    }

    // hood always position-controlled
    double hoodRotations = hoodTargetDeg / 360.0;
    hoodMotor.setControl(hoodPositionRequest.withPosition(hoodRotations));
  }

  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command closeShotCommand() {
    return runOnce(this::setCloseShot);
  }

  public Command farShotCommand() {
    return runOnce(this::setFarShot);
  }

  public Command setHoodDegreesCommand(double deg) {
    return runOnce(() -> setHoodDegrees(deg));
  }

  public Command runShooterRpsCommand(double rps) {
    return Commands.startEnd(
        () -> setShooterRps(rps),
        () -> setShooterRps(0.0),
        this);
  }

  public Command runShooterVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> setVoltage(volts),
        () -> setVoltage(0.0),
        this);
  }

  // ---------------------------------------------------------------------------
  // Util
  // ---------------------------------------------------------------------------

}
