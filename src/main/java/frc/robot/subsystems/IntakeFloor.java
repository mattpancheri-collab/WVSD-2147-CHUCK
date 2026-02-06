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

public class IntakeFloor extends SubsystemBase {

  // =========================================================================
  // STUDENT ADJUSTMENT AREA: Intake Settings
  // =========================================================================

  // CAN
  private static final int CAN_ID = 16;         
  private static final String CAN_BUS = "rio";   // or "canivore"

  // Motor behavior
  private static final InvertedValue INVERTED =
      InvertedValue.CounterClockwise_Positive;  // flip if backwards
  private static final boolean BRAKE_MODE = false;

  // Gear ratio (direct drive)
  private static final double SENSOR_TO_MECH_RATIO = 1.0; // 1:1

  // -------------------------
  // Velocity PID (Slot 0)
  // -------------------------
  /**
   * kP tuning guide (Velocity PID):
   * 1) Start with kI = 0 and kD = 0.
   * 2) Set kP very small (ex: 0.02 to 0.05).
   * 3) Command a constant speed (ex: intakeIn()) and watch velocity error:
   *    - If it reaches speed slowly / feels "lazy": increase kP.
   *    - If it overshoots and oscillates (speed bounces up/down): decrease kP.
   * 4) Your goal: fast rise to target with little/no oscillation.
   * 5) Only after kP feels good:
   *    - Add a tiny kD (ex: 0.001–0.01) if it jitters/oscillates at steady speed.
   *    - Add kI only if it NEVER quite reaches target under load (usually not needed for intakes).
   *
   * Tip: Use SmartDashboard/Shuffleboard logs of TargetRPS vs VelocityRPS.
   */
  private static final double kP = 0.15;   // TODO tune using guide above
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Feedforward (volts)
  // If you don't characterize, leaving these at 0 is fine — just tune kP.
  private static final double kS = 0.0;
  private static final double kV = 0.12;
  private static final double kA = 0.0;

  // Current limiting
  private static final boolean ENABLE_STATOR_LIMIT = true;
  private static final double STATOR_LIMIT_AMPS = 60.0;

  // Motion limits
  private static final double MAX_RPS = 90.0;          // Kraken ≈ 100 rps free
  private static final double RAMP_RPS_PER_SEC = 300;  // smooth accel

  // -------------------------
  // Intake helper speeds
  // -------------------------
  /** Positive speed = "intake in" (change sign if your mechanism is reversed). */
  private static final double IN_RPS = 45.0;     // TODO set for your intake
  /** Negative speed = "spit out". */
  private static final double OUT_RPS = -35.0;   // TODO set for your intake

  // =========================================================================
  // HARDWARE / INTERNALS (do not touch)
  // =========================================================================

  private final TalonFX motor = new TalonFX(CAN_ID, CAN_BUS);
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withSlot(0);

  private final SlewRateLimiter rpsLimiter =
      new SlewRateLimiter(RAMP_RPS_PER_SEC);

  private double targetRps = 0.0;

  // =========================================================================
  // CONSTRUCTOR
  // =========================================================================

  public IntakeFloor() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Feedback scaling
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO;

    // Output settings
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
  // HELPERS (requested)
  // =========================================================================

  /** Run intake "in" at the preset speed. */
  public void intakeIn() {
    setRps(IN_RPS);
  }

  /** Run intake "out" (eject) at the preset speed. */
  public void intakeOut() {
    setRps(OUT_RPS);
  }

  // =========================================================================
  // CONTROL API
  // =========================================================================

  /** Set intake speed in MECHANISM rotations per second */
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

  @Logged(name = "IntakeFloor/TargetRPS")
  public double getTargetRps() {
    return targetRps;
  }

  @Logged(name = "IntakeFloor/VelocityRPS")
  public double getVelocityRps() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Logged(name = "IntakeFloor/Voltage")
  public double getVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "IntakeFloor/StatorCurrent")
  public double getCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  // =========================================================================
  // PERIODIC
  // =========================================================================

  @Override
  public void periodic() {
    // Slew limit the setpoint so the intake doesn't "snap" to speed and brown out.
    double limitedRps = rpsLimiter.calculate(targetRps);
    motor.setControl(velocityRequest.withVelocity(limitedRps));
  }

  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            motor.getStatorCurrent().getValueAsDouble()
        )
    );
  }

  // =========================================================================
  // COMMANDS
  // =========================================================================

  /** Command: intake in (runs until interrupted). */
  public Command intakeInCommand() {
    return run(this::intakeIn).finallyDo(interrupted -> stop());
  }

  /** Command: intake out (runs until interrupted). */
  public Command intakeOutCommand() {
    return run(this::intakeOut).finallyDo(interrupted -> stop());
  }

  /** Command: run at a specific RPS (runs until interrupted). */
  public Command intakeCommand(double rps) {
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