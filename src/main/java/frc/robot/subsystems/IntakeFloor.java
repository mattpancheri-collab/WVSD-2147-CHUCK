package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismIds;

public class IntakeFloor extends SubsystemBase {
    private final TalonFX m_rollerMotor = new TalonFX(MechanismIds.kIntakeFloorID);

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: INTAKE SPEED
    // =========================================================================
    private static final double kIntakeSpeed = 0.6;

    public IntakeFloor() {
        // TODO: STUDENTS - Configure current limits if the intake jams often
    }

    public Command runIntakeCommand() {
        return this.runEnd(
                () -> m_rollerMotor.set(kIntakeSpeed),
                () -> m_rollerMotor.stopMotor());
    }

    public Command outtakeCommand() {
        return this.runEnd(
                () -> m_rollerMotor.set(-kIntakeSpeed),
                () -> m_rollerMotor.stopMotor());
    }
}
