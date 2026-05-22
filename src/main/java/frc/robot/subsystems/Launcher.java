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

  // =========================================================
  // Motors
  // =========================================================

  private final TalonFX shooterLeader =
      new TalonFX(CANConstants.kShooterLeaderID, kDefaultBus);

  private final TalonFX shooterFollower1 =
      new TalonFX(CANConstants.kShooterFollower1ID, kDefaultBus);

  private final TalonFX shooterFollower2 =
      new TalonFX(CANConstants.kShooterFollower2ID, kDefaultBus);

  private final TalonFX hoodMotor =
      kEnableHood ? new TalonFX(CANConstants.kHoodID, kDefaultBus) : null;

  // =========================================================
  // Shooter Control
  // =========================================================

  private enum ControlMode {
    VELOCITY,
    VOLTAGE,
    STOPPED
  }

  private ControlMode m_controlMode = ControlMode.STOPPED;

  private final VelocityVoltage shooterLeaderRequest =
      new VelocityVoltage(0).withSlot(0);

  private final Follower follower1Request =
      new Follower(CANConstants.kShooterLeaderID, MotorAlignmentValue.Aligned);

  private final Follower follower2Request =
      new Follower(CANConstants.kShooterLeaderID, MotorAlignmentValue.Opposed);

  private final SlewRateLimiter shooterSetpointLimiter =
      new SlewRateLimiter(kShooterRampRPSPerSec);

  private double shooterTargetRps = 0.0;
  private double shooterVoltageDemand = 0.0;
  private double shooterFeedForwardVolts = 0.0;

  private boolean feedReadyLatched = false;

  // =========================================================
  // Hood Control
  // =========================================================

  private final PositionVoltage hoodPositionRequest =
      new PositionVoltage(0).withSlot(0);

  private double hoodTargetDeg = 38.0;
  private boolean m_hoodActive = false;

  // =========================================================
  // Constructor
  // =========================================================

  public Launcher() {
    configureShooterLeader();
    configureShooterFollower(shooterFollower1);
    configureShooterFollower(shooterFollower2);

    if (kEnableHood && hoodMotor != null) {
      configureHood();
    }

    applyFollowerMode();

    // Start safe.
    stopShooter();
    stopHood();
  }

  // =========================================================
  // Configuration
  // =========================================================

  private void applyFollowerMode() {
    shooterFollower1.setControl(follower1Request);
    shooterFollower2.setControl(follower2Request);
  }

  private void configureShooterLeader() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

    // This means hood position units are MECHANISM rotations,
    // not motor rotations. So 90 degrees = 0.25 rotations.
    cfg.Feedback.SensorToMechanismRatio = kHoodGearRatio;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs s0 = cfg.Slot0;
    s0.kP = kHoodP;
    s0.kI = kHoodI;
    s0.kD = kHoodD;

    // Arm_Cosine is useful if the hood behaves like a rotating arm.
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

  // =========================================================
  // Shooter Methods
  // =========================================================

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
    shooterTargetRps = 0.0;
    shooterFeedForwardVolts = 0.0;
    shooterSetpointLimiter.reset(0.0);

    shooterVoltageDemand =
        MathUtil.clamp(kShooterPolarity * volts, -12.0, 12.0);

    m_controlMode = ControlMode.VOLTAGE;
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

  /**
   * Stops ONLY the shooter.
   * This does NOT stop the hood.
   */
  public void stopShooter() {
    shooterVoltageDemand = 0.0;
    shooterTargetRps = 0.0;
    shooterFeedForwardVolts = 0.0;
    shooterSetpointLimiter.reset(0.0);

    shooterLeader.setVoltage(0.0);
    applyFollowerMode();

    m_controlMode = ControlMode.STOPPED;
    feedReadyLatched = false;
  }

  /**
   * Full launcher stop.
   * Use this only when you truly want both shooter and hood stopped.
   */
  public void stop() {
    stopShooter();
    stopHood();
  }

  // =========================================================
  // Hood Methods
  // =========================================================

  public void setHoodDegrees(double deg) {
    hoodTargetDeg = MathUtil.clamp(deg, kHoodMinDeg, kHoodMaxDeg);
    m_hoodActive = kEnableHood && hoodMotor != null;

    if (m_hoodActive) {
      double hoodRotations = hoodTargetDeg / 360.0;
      hoodMotor.setControl(hoodPositionRequest.withPosition(hoodRotations));
    }
  }

  public void stopHood() {
    m_hoodActive = false;

    if (kEnableHood && hoodMotor != null) {
      hoodMotor.setVoltage(0.0);
    }
  }

  public void disableHood() {
    stopHood();
  }

  /**
   * Sets the current hood position to 0 degrees.
   * Only use this when the hood is physically at your chosen zero position.
   */
  public void zeroHood() {
    if (kEnableHood && hoodMotor != null) {
      hoodMotor.setPosition(0.0);
    }
  }

  /**
   * Temporary test method.
   * This bypasses position control and simply applies voltage to the hood motor.
   */
  public void setHoodTestVoltage(double volts) {
    if (kEnableHood && hoodMotor != null) {
      m_hoodActive = true;
      hoodMotor.setVoltage(MathUtil.clamp(volts, -4.0, 4.0));
    }
  }

  // =========================================================
  // Shooter Getters
  // =========================================================

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

  public boolean shooterReadyForFeed() {
    double error = Math.abs(getShooterErrorRps());

    if (feedReadyLatched) {
      if (error >= kShooterFeedDisableErrorRps) {
        feedReadyLatched = false;
      }
    } else {
      if (error <= kShooterFeedEnableErrorRps) {
        feedReadyLatched = true;
      }
    }

    return feedReadyLatched;
  }

  // =========================================================
  // Hood Getters
  // =========================================================

  public double getHoodTargetDeg() {
    return hoodTargetDeg;
  }

  public double getHoodPositionDeg() {
    if (!kEnableHood || hoodMotor == null) {
      return 0.0;
    }

    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public double getHoodErrorDeg() {
    return hoodTargetDeg - getHoodPositionDeg();
  }

  public double getHoodAppliedVolts() {
    if (!kEnableHood || hoodMotor == null) {
      return 0.0;
    }

    return hoodMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getHoodStatorCurrentAmps() {
    if (!kEnableHood || hoodMotor == null) {
      return 0.0;
    }

    return hoodMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getHoodSupplyCurrentAmps() {
    if (!kEnableHood || hoodMotor == null) {
      return 0.0;
    }

    return hoodMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isHoodEnabled() {
    return kEnableHood && hoodMotor != null;
  }

  public boolean isHoodActive() {
    return m_hoodActive;
  }

  // =========================================================
  // Periodic
  // =========================================================

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

    // Shooter dashboard
    SmartDashboard.putString("Launcher/ControlMode", getControlModeName());
    SmartDashboard.putNumber("Launcher/TargetRPS", shooterTargetRps);
    SmartDashboard.putNumber("Launcher/ActualRPS", getShooterLeaderVelocityRps());
    SmartDashboard.putNumber("Launcher/ErrorRPS", getShooterErrorRps());
    SmartDashboard.putBoolean("Launcher/AtSpeed", shooterAtSpeed());
    SmartDashboard.putBoolean("Launcher/ReadyForFeed", shooterReadyForFeed());
    SmartDashboard.putNumber("Launcher/ShooterVoltage", getShooterAppliedVolts());
    SmartDashboard.putNumber("Launcher/ShooterStatorCurrent", getShooterStatorCurrentAmps());

    // Hood dashboard
    SmartDashboard.putBoolean("Launcher/HoodEnabled", isHoodEnabled());
    SmartDashboard.putBoolean("Launcher/HoodActive", m_hoodActive);
    SmartDashboard.putNumber("Launcher/HoodTargetDeg", getHoodTargetDeg());
    SmartDashboard.putNumber("Launcher/HoodPositionDeg", getHoodPositionDeg());
    SmartDashboard.putNumber("Launcher/HoodErrorDeg", getHoodErrorDeg());
    SmartDashboard.putNumber("Launcher/HoodVoltage", getHoodAppliedVolts());
    SmartDashboard.putNumber("Launcher/HoodStatorCurrent", getHoodStatorCurrentAmps());
    SmartDashboard.putNumber("Launcher/HoodSupplyCurrent", getHoodSupplyCurrentAmps());
  }

  // =========================================================
  // Commands
  // =========================================================

  /**
   * Stops both shooter and hood.
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /**
   * Stops only the shooter.
   * Use this when ending shooter commands so the hood does not get shut off.
   */
  public Command stopShooterCommand() {
    return runOnce(this::stopShooter);
  }

  /**
   * Stops only the hood.
   */
  public Command stopHoodCommand() {
    return Commands.runOnce(this::stopHood);
  }

  public Command closeShotCommand() {
    return runOnce(this::setCloseShot);
  }

  public Command farShotCommand() {
    return runOnce(this::setFarShot);
  }

  /**
   * Important:
   * This uses Commands.runOnce instead of this.runOnce.
   * That means it does NOT require the whole Launcher subsystem.
   * This helps prevent the hood button from being blocked by shooter commands.
   */
  public Command setHoodDegreesCommand(double deg) {
    return Commands.runOnce(() -> setHoodDegrees(deg));
  }

  public Command disableHoodCommand() {
    return Commands.runOnce(this::disableHood);
  }

  public Command zeroHoodCommand() {
    return Commands.runOnce(this::zeroHood);
  }

  /**
   * Shooter command that stops only the shooter when released.
   * It no longer shuts off the hood.
   */
  public Command runShooterRpsCommand(double rps) {
    return Commands.startEnd(
        () -> setShooterRps(rps),
        this::stopShooter,
        this);
  }

  /**
   * Shooter voltage command that stops only the shooter when released.
   * It no longer shuts off the hood.
   */
  public Command runShooterVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> setVoltage(volts),
        this::stopShooter,
        this);
  }

  /**
   * Temporary test command for hood movement.
   * Hold the button and the hood should move with direct voltage.
   * Release the button and the hood stops.
   */
  public Command testHoodVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> setHoodTestVoltage(volts),
        this::stopHood);
  }
}