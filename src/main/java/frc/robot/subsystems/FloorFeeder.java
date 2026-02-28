package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.BusConstants.kDefaultBus;
import static frc.robot.Constants.FloorFeederConstants.*;
import frc.robot.Constants.CANConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorFeeder extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS
  // =========================================================================
  private final TalonFX motor = new TalonFX(CANConstants.kFloorFeederID, kDefaultBus);

  // Closed-loop velocity request
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(kRampRPSPerSec);

  // =========================================================================
  // STATE
  // =========================================================================
  private double targetRps = 0.0;

  // Voltage override for testing controller
  private boolean voltageOverride = false;
  private double voltageDemand = 0.0;

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================
  public FloorFeeder() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // flip if needed

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    // Feedforward:
    // For tuning, see Constants.java lines 117-119
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = kStatorLimitAmps;

    motor.getConfigurator().apply(config);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  public void feederIn() {
    setRps(kFeedInRPS);
  }

  public void feederOut() {
    setRps(kFeedOutRPS);
  }

  /** Velocity mode (RPS). Exits voltage override. */
  public void setRps(double rps) {
    voltageOverride = false;
    voltageDemand = 0.0;

    targetRps = MathUtil.clamp(rps, -kMaxRPS, kMaxRPS);
  }

  /**
   * Voltage mode (testing). Prevents periodic() from running velocity control.
   */
  public void setVoltage(double volts) {
    voltageOverride = true;
    voltageDemand = MathUtil.clamp(volts, -12.0, 12.0);

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

  // =========================================================================
  // SIMPLE GETTERS
  // =========================================================================

  public double getTargetRps() {
    return targetRps;
  }

  public double getVelocityRps() {
    return motor.getVelocity().getValueAsDouble();
  }

  public double getAppliedVolts() {
    return motor.getMotorVoltage().getValueAsDouble();
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
    if (voltageOverride) {
      motor.setVoltage(voltageDemand);
      return;
    }

    double limitedRps = rpsLimiter.calculate(targetRps);
    motor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  // =========================================================================
  // COMMANDS (no finallyDo)
  // =========================================================================

  public Command feederInCommand() {
    return Commands.startEnd(this::feederIn, this::stop, this);
  }

  public Command feederOutCommand() {
    return Commands.startEnd(this::feederOut, this::stop, this);
  }

  public Command feederCommand(double rps) {
    return Commands.startEnd(() -> setRps(rps), this::stop, this);
  }

  public Command feederVoltageCommand(double volts) {
    return Commands.startEnd(() -> setVoltage(volts), () -> setVoltage(0.0), this);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

}
