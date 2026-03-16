package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

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

import frc.robot.Constants.IntakeFloorConstants;
import frc.robot.Constants.BusConstants;
import frc.robot.Constants.CANConstants;

public class IntakeGround extends SubsystemBase {

  private static final double kEpsilon = 1e-4;

  private final TalonFX intakeMotor = new TalonFX(CANConstants.kIntakeID, BusConstants.kDefaultBus);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(IntakeFloorConstants.kRampRPSPerSec);

  private double targetRps = 0.0;
  private boolean voltageOverride = false;
  private double voltageDemand = 0.0;

  public IntakeGround() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = IntakeFloorConstants.kP;
    slot0.kI = IntakeFloorConstants.kI;
    slot0.kD = IntakeFloorConstants.kD;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = IntakeFloorConstants.kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = IntakeFloorConstants.kStatorLimitAmps;

    intakeMotor.getConfigurator().apply(config);
  }

  public void intakeIn() {
    setRps(IntakeFloorConstants.kIntakeInRPS);
  }

  public void intakeOut() {
    setRps(IntakeFloorConstants.kIntakeOutRPS);
  }

  public void setRps(double rps) {
    voltageOverride = false;
    voltageDemand = 0.0;
    targetRps = MathUtil.clamp(rps, -IntakeFloorConstants.kMaxRPS, IntakeFloorConstants.kMaxRPS);
  }

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
    intakeMotor.setVoltage(0.0);
  }

  public double getTargetRps() {
    return targetRps;
  }

  public double getVelocityRps() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public double getAppliedVolts() {
    return intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  public boolean isVoltageOverride() {
    return voltageOverride;
  }

  public double getVoltageDemand() {
    return voltageDemand;
  }

  @Override
  public void periodic() {
    if (voltageOverride) {
      if (Math.abs(voltageDemand) > kEpsilon) {
        intakeMotor.setVoltage(voltageDemand);
      }
      return;
    }

    if (Math.abs(targetRps) <= kEpsilon) {
      return;
    }

    double limitedRps = rpsLimiter.calculate(targetRps);
    intakeMotor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  public Command intakeInCommand() {
    return Commands.startEnd(this::intakeIn, this::stop, this);
  }

  public Command intakeOutCommand() {
    return Commands.startEnd(this::intakeOut, this::stop, this);
  }

  public Command intakeCommand(double rps) {
    return Commands.startEnd(() -> setRps(rps), this::stop, this);
  }

  public Command testMotorCommand(double rps) {
    return Commands.startEnd(() -> setRps(rps), this::stop, this);
  }

  public Command testVoltageCommand(double volts) {
    return Commands.startEnd(() -> setVoltage(volts), this::stop, this);
}

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}