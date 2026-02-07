package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LaunchFeederConstants.*;
import static frc.robot.Constants.CANBus.kDefaultBus;

public class LaunchFeeder extends SubsystemBase {

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  private final TalonFX motor = new TalonFX(kFeederID, kDefaultBus);
  private final CANrange canrange = new CANrange(kCANrangeID, kDefaultBus);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(kRampRPSPerSec);

  private final Debouncer ballDebouncer = new Debouncer(kBallDebounceSeconds, Debouncer.DebounceType.kRising);

  private double targetRps = 0.0;

  public LaunchFeeder() {
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
    slot0.kV = 0.12; // Static for now or tuneable
    slot0.kA = 0.0;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = kStatorLimitAmps;

    motor.getConfigurator().apply(config);
  }

  // =========================================================================
  // SENSOR (CANrange)
  // =========================================================================

  @Logged(name = "LauncherFeeder/CANrangeDistanceM")
  public double getDistanceMeters() {
    return canrange.getDistance().getValueAsDouble();
  }

  @Logged(name = "LauncherFeeder/BallDetected")
  public boolean hasBall() {
    double d = getDistanceMeters();
    boolean raw = d > 0.0 && d < kBallDetectionDistanceMeters;
    return ballDebouncer.calculate(raw);
  }

  // =========================================================================
  // HELPERS
  // =========================================================================

  public void feederIn() {
    setRps(kFeedInRPS);
  }

  public void feederOut() {
    setRps(kFeedOutRPS);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  public void setRps(double rps) {
    targetRps = clamp(rps, -kMaxRPS, kMaxRPS);
  }

  public void stop() {
    targetRps = 0.0;
    motor.stopMotor();
  }

  // =========================================================================
  // TELEMETRY
  // =========================================================================

  @Logged(name = "LauncherFeeder/TargetRPS")
  public double getTargetRps() {
    return targetRps;
  }

  @Logged(name = "LauncherFeeder/VelocityRPS")
  public double getVelocityRps() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Logged(name = "LauncherFeeder/AppliedVolts")
  public double getVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "LauncherFeeder/StatorCurrentA")
  public double getCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    if (kAutoStopOnBall && targetRps > 0.0 && hasBall()) {
      stop();
      return;
    }

    double limitedRps = rpsLimiter.calculate(targetRps);
    motor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  public Command feedUntilBallCommand() {
    return run(this::feederIn)
        .until(this::hasBall)
        .finallyDo(interrupted -> stop());
  }

  public Command feederInCommand() {
    return feederCommand(kFeedInRPS);
  }

  public Command feederOutCommand() {
    return feederCommand(kFeedOutRPS);
  }

  /** ✅ NEW: matches your RobotContainer "40% constant" style */
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
