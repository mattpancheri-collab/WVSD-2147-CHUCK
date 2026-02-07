package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismIds;

public class Climber extends SubsystemBase {
    private final TalonFX m_climberMotor = new TalonFX(MechanismIds.kClimberID);

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: CLIMB SPEED Here
    // =========================================================================
    private static final double kClimbPower = 0.8;

    public Climber() {
        // SAFETY: Brake mode is CRITICAL for climbers to prevent falling when disabled
        // Without brake mode, the robot will drop when the motor is not powered!
        var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        m_climberMotor.getConfigurator().apply(cfg);
    }

    public Command climbCommand() {
        return this.runEnd(
                () -> m_climberMotor.set(kClimbPower),
                () -> m_climberMotor.stopMotor());
    }

    public Command reverseClimbCommand() {
        return this.runEnd(
                () -> m_climberMotor.set(-kClimbPower),
                () -> m_climberMotor.stopMotor());
    }

    /**
     * Test climber motor at specific percent output (-1.0 to 1.0) for hardware
     * validation.
     * Use this to verify motor wiring, direction, and brake mode.
     */
    public Command testMotorCommand(double percentOutput) {
        return this.runEnd(
                () -> m_climberMotor.set(percentOutput),
                () -> m_climberMotor.stopMotor());
    }
}
