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

public class LaunchFeeder extends SubsystemBase {

  // =========================================================================
  // STUDENT ADJUSTMENT AREA: Launcher Feeder Settings
  // =========================================================================

  private static final int FEEDER_CAN_ID = 13;
  private static final int CANRANGE_CAN_ID = 30;
  private static final String CAN_BUS = "rio";

  private static final InvertedValue INVERTED =
      InvertedValue.CounterClockwise_Positive;
  private static final boolean BRAKE_MODE = false;

  private static final double SENSOR_TO_MECH_RATIO = 1.0;

  private static final double kP = 0.12;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kS = 0.0;
  private static final double kV = 0.12;
  private static final double kA = 0.0;

  private static final boolean ENABLE_STATOR_LIMIT = true;
  private static final double STATOR_LIMIT_AMPS = 60.0;

  private static final double MAX_RPS = 80.0;
  private static final double RAMP_RPS_PER_SEC = 300;

  private static final double FEED_IN_RPS = 40.0;
  private static final double FEED_OUT_RPS = -30.0;

  private static final double BALL_DETECT_DISTANCE_M = 0.12;
  private static final double BALL_DETECT_DEBOUNCE_SEC = 0.05;
  private static final boolean AUTO_STOP_ON_BALL = true;

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  private final TalonFX motor = new TalonFX(FEEDER_CAN_ID, CAN_BUS);
  private final CANrange canrange = new CANrange(CANRANGE_CAN_ID, CAN_BUS);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(RAMP_RPS_PER_SEC);

  private final Debouncer ballDebouncer =
      new Debouncer(BALL_DETECT_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);

  private double targetRps = 0.0;

  public LaunchFeeder() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO;

    config.MotorOutput.NeutralMode =
        BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted = INVERTED;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = ENABLE_STATOR_LIMIT;
    currentLimits.StatorCurrentLimit = STATOR_LIMIT_AMPS;

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
    boolean raw = d > 0.0 && d < BALL_DETECT_DISTANCE_M;
    return ballDebouncer.calculate(raw);
  }

  // =========================================================================
  // HELPERS
  // =========================================================================

  public void feederIn() {
    setRps(FEED_IN_RPS);
  }

  public void feederOut() {
    setRps(FEED_OUT_RPS);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  public void setRps(double rps) {
    targetRps = clamp(rps, -MAX_RPS, MAX_RPS);
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
    if (AUTO_STOP_ON_BALL && targetRps > 0.0 && hasBall()) {
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
    return run(this::feederIn).finallyDo(interrupted -> stop());
  }

  public Command feederOutCommand() {
    return run(this::feederOut).finallyDo(interrupted -> stop());
  }

  /** ✅ NEW: matches your RobotContainer "40% constant" style */
  public Command feederCommand(double rps) {
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
