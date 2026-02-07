package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CANBus;

/**
 * Climber subsystem:
 * - Controls the robot's climb mechanism.
 * - Single motor control with brake mode enabled for safety.
 */
public class Climber extends SubsystemBase {

    // =========================================================================
    // HARDWARE / INTERNALS
    // =========================================================================

    /** ✅ Climber Motor (TalonFX) */
    private final TalonFX motor = new TalonFX(ClimberConstants.kClimberID, CANBus.kDefaultBus);

    public Climber() {
        configureMotor();
        stop();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // CRITICAL: Set to brake mode so the robot doesn't slide down after power is
        // cut.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.StatorCurrentLimitEnable = ClimberConstants.kEnableStatorLimit;
        currentLimits.StatorCurrentLimit = ClimberConstants.kStatorLimitAmps;

        motor.getConfigurator().apply(config);
    }

    // =========================================================================
    // CONTROL API
    // =========================================================================

    /** 👋 Stop the climber motor. */
    public void stop() {
        motor.stopMotor();
    }

    /** 🧗 Climb UP at default power. */
    public void climb() {
        motor.set(ClimberConstants.kClimbPower);
    }

    /** 🧗 Reverse the climber. */
    public void reverseClimb() {
        motor.set(ClimberConstants.kClimbReversePower);
    }

    // =========================================================================
    // COMMANDS
    // =========================================================================

    /** 🎯 Command to climb. */
    public Command climbCommand() {
        return run(this::climb).finallyDo(interrupted -> stop());
    }

    /** 🎯 Command to reverse climb. */
    public Command reverseClimbCommand() {
        return run(this::reverseClimb).finallyDo(interrupted -> stop());
    }

    /** 👋 Command to stop the climber. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
