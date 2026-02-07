package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FloorFeederConstants.*;
import static frc.robot.Constants.CANBus.kDefaultBus;

public class FloorFeeder extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  private final TalonFX motor = new TalonFX(kFeederID, kDefaultBus);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(kRampRPSPerSec);

  private double targetRps = 0.0;

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
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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

  // =========================================================================
  // HELPERS
  // =========================================================================

  /** Feed game piece forward at preset speed. */
  public void feederIn() {
    setRps(kFeedInRPS);
  }

  /** Reverse feeder (spit out / clear jam) at preset speed. */
  public void feederOut() {
    setRps(kFeedOutRPS);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** Set feeder speed in MECHANISM rotations per second. */
  public void setRps(double rps) {
    this.targetRps = clamp(rps, -kMaxRPS, kMaxRPS);
  }

  public void stop() {
    targetRps = 0.0;
    motor.stopMotor();
  }

  // =========================================================================
  // TELEMETRY
  // =========================================================================

  @Logged(name = "FloorFeeder/TargetRPS")
  public double getTargetRps() {
    return targetRps;
  }

  @Logged(name = "FloorFeeder/VelocityRPS")
  public double getVelocityRps() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Logged(name = "FloorFeeder/Voltage")
  public double getVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "FloorFeeder/StatorCurrent")
  public double getCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    // Slew limit setpoint so we don't brown out / shock load belts.
    double limitedRps = rpsLimiter.calculate(targetRps);

    // Velocity PID setpoint is in mechanism rotations/sec (after
    // SensorToMechanismRatio).
    motor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  @Override
  public void simulationPeriodic() {
    // Lightweight battery sag effect (not a full mechanism sim).
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            motor.getStatorCurrent().getValueAsDouble()));
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  /** Command: feeder in (runs until interrupted). */
  public Command feederInCommand() {
    return run(this::feederIn).finallyDo(interrupted -> stop());
  }

  /** Command: feeder out (runs until interrupted). */
  public Command feederOutCommand() {
    return run(this::feederOut).finallyDo(interrupted -> stop());
  }

  /** Command: run at a specific RPS (runs until interrupted). */
  public Command feederCommand(double rps) {
    return run(() -> setRps(rps)).finallyDo(interrupted -> stop());
  }

  /**
   * Test feeder motor at specific RPS for hardware validation.
   * Use this to verify motor wiring and direction.
   */
  public Command testMotorCommand(double rps) {
    return run(() -> setRps(rps)).finallyDo(interrupted -> stop());
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  // =========================================================================
  // UTIL
  // =========================================================================

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}