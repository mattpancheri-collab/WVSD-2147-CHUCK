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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final VelocityVoltage shooterLeaderRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withSlot(0);

  private final Follower follower1Request =
      new Follower(CANConstants.kShooterLeaderID, MotorAlignmentValue.Aligned);

  private final Follower follower2Request =
      new Follower(CANConstants.kShooterLeaderID, MotorAlignmentValue.Opposed);

  private final SlewRateLimiter shooterSetpointLimiter =
      new SlewRateLimiter(kShooterRampRPSPerSec);

  private double shooterTargetRps = 0.0; // always stored positive
  private double shooterVoltageDemand = 0.0;
  private double shooterFeedForwardVolts = 0.0;

  private double hoodTargetDeg = 38.0;
  private boolean m_hoodActive = false;

  public Launcher() {
    configureShooterLeader();
    configureShooterFollower(shooterFollower1);
    configureShooterFollower(shooterFollower2);
    configureHood();

    shooterFollower1.setControl(follower1Request);
    shooterFollower2.setControl(follower2Request);

    stop();
  }

  private void configureShooterLeader() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = kShooterP;
    s0.kI = kShooterI;
    s0.kD = kShooterD;
    s0.kS = kShooterS;
    s0.kV = kShooterV;
    s0.kA = kShooterA;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.SupplyCurrentLimitEnable = true;
    cl.SupplyCurrentLimit = 80.0;
    cl.StatorCurrentLimitEnable = true;
    cl.StatorCurrentLimit = 160.0;

    shooterLeader.getConfigurator().apply(cfg);
  }

  private void configureShooterFollower(TalonFX motor) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    CurrentLimitsConfigs cl = cfg.CurrentLimits;
    cl.SupplyCurrentLimitEnable = true;
    cl.SupplyCurrentLimit = 80.0;
    cl.StatorCurrentLimitEnable = true;
    cl.StatorCurrentLimit = 160.0;

    motor.getConfigurator().apply(cfg);
  }

  private void configureHood() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = kHoodGearRatio;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

  public void setShooterRps(double rps) {
    setShooterRps(rps, 0.0);
  }

  public void setShooterRps(double rps, double extraFeedForwardVolts) {
    shooterVoltageDemand = 0.0;
    shooterTargetRps = MathUtil.clamp(rps, 0.0, kShooterMaxRPS);
    shooterFeedForwardVolts = MathUtil.clamp(extraFeedForwardVolts, -2.0, 2.0);
    m_controlMode = ControlMode.VELOCITY;
  }

  public void setVoltage(double volts) {
    System.out.println("[Launcher] setVoltage: " + volts);

    shooterTargetRps = 0.0;
    shooterFeedForwardVolts = 0.0;
    shooterSetpointLimiter.reset(0.0);

    shooterVoltageDemand = MathUtil.clamp(kShooterPolarity * volts, -12.0, 12.0);
    m_controlMode = ControlMode.VOLTAGE;
  }

  public void setHoodDegrees(double deg) {
    hoodTargetDeg = MathUtil.clamp(deg, kHoodMinDeg, kHoodMaxDeg);
    m_hoodActive = true;
  }

  public void disableHood() {
    m_hoodActive = false;
  }

  public void stop() {
    shooterVoltageDemand = 0.0;
    shooterTargetRps = 0.0;
    shooterFeedForwardVolts = 0.0;
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
  }

  public void setFarShot() {
    setShooterRps(kShooterFarRPS);
  }

  public double getShooterTargetRps() {
    return shooterTargetRps;
  }

  public double getShooterLeaderVelocityRps() {
    return Math.abs(shooterLeader.getVelocity().getValueAsDouble());
  }

  public double getShooterErrorRps() {
    return shooterTargetRps - getShooterLeaderVelocityRps();
  }

  public boolean shooterAtSpeed() {
    return Math.abs(getShooterErrorRps()) <= kShooterReadyToleranceRps;
  }

  public double getShooterAppliedVolts() {
    return shooterLeader.getMotorVoltage().getValueAsDouble();
  }

  public double getShooterSupplyCurrentAmps() {
    return shooterLeader.getSupplyCurrent().getValueAsDouble();
  }

  public double getShooterStatorCurrentAmps() {
    return shooterLeader.getStatorCurrent().getValueAsDouble();
  }

  public double getShooterFeedForwardVolts() {
    return shooterFeedForwardVolts;
  }

  public String getControlModeName() {
    return m_controlMode.name();
  }

  public double getHoodTargetDeg() {
    return hoodTargetDeg;
  }

  public double getHoodPositionDeg() {
    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case VELOCITY:
        double limitedSignedRps =
            shooterSetpointLimiter.calculate(kShooterPolarity * shooterTargetRps);

        shooterLeader.setControl(
            shooterLeaderRequest
                .withVelocity(limitedSignedRps)
                .withFeedForward(shooterFeedForwardVolts));
        break;

      case VOLTAGE:
        shooterLeader.setVoltage(shooterVoltageDemand);
        break;

      case STOPPED:
      default:
        break;
    }

    if (m_hoodActive) {
      double hoodRotations = hoodTargetDeg / 360.0;
      hoodMotor.setControl(hoodPositionRequest.withPosition(hoodRotations));
    } else {
      hoodMotor.setVoltage(0.0);
    }

    SmartDashboard.putNumber("Launcher/TargetRPS", shooterTargetRps);
    SmartDashboard.putNumber("Launcher/ActualRPS", getShooterLeaderVelocityRps());
    SmartDashboard.putNumber("Launcher/ErrorRPS", getShooterErrorRps());
    SmartDashboard.putBoolean("Launcher/AtSpeed", shooterAtSpeed());

    SmartDashboard.putNumber("Launcher/AppliedVolts", getShooterAppliedVolts());
    SmartDashboard.putNumber("Launcher/FFVolts", getShooterFeedForwardVolts());
    SmartDashboard.putNumber("Launcher/SupplyCurrentAmps", getShooterSupplyCurrentAmps());
    SmartDashboard.putNumber("Launcher/StatorCurrentAmps", getShooterStatorCurrentAmps());

    SmartDashboard.putString("Launcher/ControlMode", getControlModeName());

    SmartDashboard.putNumber("Launcher/HoodTargetDeg", hoodTargetDeg);
    SmartDashboard.putNumber("Launcher/HoodActualDeg", getHoodPositionDeg());
    SmartDashboard.putBoolean("Launcher/HoodActive", m_hoodActive);
  }

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

  public Command disableHoodCommand() {
    return runOnce(this::disableHood);
  }

  public Command runShooterRpsCommand(double rps) {
    return Commands.startEnd(
        () -> setShooterRps(rps),
        this::stop,
        this);
  }

  public Command runShooterVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> setVoltage(volts),
        this::stop,
        this);
  }
}