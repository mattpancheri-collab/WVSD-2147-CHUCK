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

public class FloorFeeder extends SubsystemBase {

  // =========================================================================
  // STUDENT ADJUSTMENT AREA: Floor Feeder Settings
  // =========================================================================

  // CAN
  private static final int CAN_ID = 14;          // TODO set
  private static final String CAN_BUS = "rio";   // or "canivore"

  // Motor behavior (Phoenix 6 uses enum, not boolean)
  private static final InvertedValue INVERTED =
      InvertedValue.CounterClockwise_Positive;  // flip if backwards
  private static final boolean BRAKE_MODE = false;

  // Gear ratio
  // Set to (motor rotations) / (mechanism rotations).
  // 1:1 direct drive => 1.0
  private static final double SENSOR_TO_MECH_RATIO = 1.0; // TODO set if geared

  // -------------------------
  // Velocity PID (Slot 0)
  // -------------------------
  /**
   * kP tuning guide (Velocity PID):
   * 1) Start with kI = 0 and kD = 0.
   * 2) Start kP small (ex: 0.02–0.08).
   * 3) Command a constant speed (ex: feederIn()) and watch VelocityRPS vs TargetRPS:
   *    - Too slow to reach target / big steady error => increase kP.
   *    - Overshoot/oscillation (speed bounces) => decrease kP.
   * 4) Goal: quick rise, minimal overshoot, stable steady speed.
   * 5) If it “buzzes” around target, add tiny kD (ex: 0.001–0.01).
   * 6) kI is rarely needed on feeders; only add if it *never* reaches target under load.
   */
  private static final double kP = 0.18;   // TODO tune
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Feedforward (volts) - optional
  private static final double kS = 0.0;
  private static final double kV = 0.12;
  private static final double kA = 0.0;

  // Current limiting
  private static final boolean ENABLE_STATOR_LIMIT = true;
  private static final double STATOR_LIMIT_AMPS = 60.0;

  // Motion limits
  private static final double MAX_RPS = 90.0;          // Kraken ≈ 100 rps free
  private static final double RAMP_RPS_PER_SEC = 350;  // smooth accel

  // Preset helper speeds
  /** Positive = feed toward shooter/indexer (flip sign if your mechanism is reversed). */
  private static final double FEED_IN_RPS = 40.0;   // TODO set
  /** Negative = reverse / clear jam. */
  private static final double FEED_OUT_RPS = -30.0; // TODO set

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  private final TalonFX motor = new TalonFX(CAN_ID, CAN_BUS);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final SlewRateLimiter rpsLimiter = new SlewRateLimiter(RAMP_RPS_PER_SEC);

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

    // Feedback scaling
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO;

    // Output
    config.MotorOutput.NeutralMode =
        BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted = INVERTED;

    // Slot 0 PID + FF
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = ENABLE_STATOR_LIMIT;
    currentLimits.StatorCurrentLimit = STATOR_LIMIT_AMPS;

    motor.getConfigurator().apply(config);
  }

  // =========================================================================
  // HELPERS
  // =========================================================================

  /** Feed game piece forward at preset speed. */
  public void feederIn() {
    setRps(FEED_IN_RPS);
  }

  /** Reverse feeder (spit out / clear jam) at preset speed. */
  public void feederOut() {
    setRps(FEED_OUT_RPS);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** Set feeder speed in MECHANISM rotations per second. */
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

    // Velocity PID setpoint is in mechanism rotations/sec (after SensorToMechanismRatio).
    motor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  @Override
  public void simulationPeriodic() {
    // Lightweight battery sag effect (not a full mechanism sim).
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            motor.getStatorCurrent().getValueAsDouble()
        )
    );
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