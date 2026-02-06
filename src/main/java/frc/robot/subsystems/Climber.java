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
        // TODO: STUDENTS - Ensure brake mode is ON so we don't fall!
        // var cfg = new TalonFXConfiguration();
        // cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // m_climberMotor.getConfigurator().apply(cfg);
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
}
