package frc.robot.subsystems;

import static frc.robot.Constants.BusConstants.kDefaultBus;
import static frc.robot.Constants.LaunchFeederConstants.*;
import frc.robot.Constants.CANConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchFeeder extends SubsystemBase {

  private final TalonFX motor = new TalonFX(CANConstants.kLaunchFeederID, kDefaultBus);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(kRampRPSPerSec);

  private final CANrange canrange = kEnableCANrange ? new CANrange(CANConstants.kCANrangeID, kDefaultBus) : null;

  private final Debouncer ballDebouncer = new Debouncer(kBallDebounceSeconds, Debouncer.DebounceType.kRising);

  private double targetRps = 0.0;

  private boolean voltageOverride = false;
  private double voltageDemand = 0.0;

  // DEBUG: keep prints from spamming every 20ms
  private int debugCounter = 0;

  public LaunchFeeder() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    slot0.kS = 0.0;
    slot0.kV = 0.12;
    slot0.kA = 0.0;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = kStatorLimitAmps;

    motor.getConfigurator().apply(config);
  }

  // SENSOR (CANrange)
  public double getDistanceMeters() {
    if (canrange == null)
      return Double.NaN;
    return canrange.getDistance().getValueAsDouble();
  }

  public boolean hasBall() {
    if (canrange == null)
      return false;

    double d = getDistanceMeters();
    boolean raw = d > 0.0 && d < kBallDetectionDistanceMeters;
    return ballDebouncer.calculate(raw);
  }

  // HELPERS
  public void feederIn() {
    setRps(kFeedInRPS);
  }

  public void feederOut() {
    setRps(kFeedOutRPS);
  }

  // CONTROL API
  public void setRps(double rps) {
    voltageOverride = false;
    voltageDemand = 0.0;

    targetRps = clamp(rps, -kMaxRPS, kMaxRPS);
  }

  public void setVoltage(double volts) {
    // DEBUG: proves the button/command is actually calling this
    System.out.println("[LaunchFeeder] setVoltage called: " + volts);

    voltageOverride = true;
    voltageDemand = clamp(volts, -12.0, 12.0);

    targetRps = 0.0;
    rpsLimiter.reset(0.0);
  }

  public void stop() {
    voltageOverride = false;
    voltageDemand = 0.0;
    targetRps = 0.0;

    rpsLimiter.reset(0.0);

    motor.setVoltage(0.0);
  }

  // SIMPLE GETTERS
  public double getTargetRps() {
    return targetRps;
  }

  public double getVelocityRps() {
    return motor.getVelocity().getValueAsDouble();
  }

  public double getAppliedVolts() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  public double getStatorCurrentA() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  public boolean isVoltageOverride() {
    return voltageOverride;
  }

  public double getVoltageDemand() {
    return voltageDemand;
  }

  @Override
  public void periodic() {
    if (!voltageOverride && canrange != null && kAutoStopOnBall && targetRps > 0.0 && hasBall()) {
      stop();
      return;
    }

    if (voltageOverride) {
      motor.setVoltage(voltageDemand);

      // DEBUG: print ~5x/sec
      debugCounter++;
      if (debugCounter % 10 == 0) {
        System.out.println("[LaunchFeeder] demand=" + voltageDemand
            + " applied=" + motor.getMotorVoltage().getValueAsDouble()
            + " current=" + motor.getStatorCurrent().getValueAsDouble());

        System.out.println("[LaunchFeeder] enabled=" + edu.wpi.first.wpilibj.DriverStation.isEnabled()
            + " brownout=" + edu.wpi.first.wpilibj.RobotController.isBrownedOut()
            + " batt=" + edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());

      }
      return;
    }

    double limitedRps = rpsLimiter.calculate(targetRps);
    motor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  // COMMANDS
  public Command feedUntilBallCommand() {
    return Commands.startEnd(this::feederIn, this::stop, this).until(this::hasBall);
  }

  public Command feederInCommand() {
    return feederCommand(kFeedInRPS);
  }

  public Command feederOutCommand() {
    return feederCommand(kFeedOutRPS);
  }

  public Command feederCommand(double rps) {
    return Commands.startEnd(() -> setRps(rps), this::stop, this);
  }

  public Command feederVoltageCommand(double volts) {
    return Commands.startEnd(() -> setVoltage(volts), this::stop, this);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
