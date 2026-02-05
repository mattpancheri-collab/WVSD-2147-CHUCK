package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismIds;

public class FloorFeeder extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MechanismIds.kFloorFeederID);

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: SPEED
    // =========================================================================
    private static final double kRunSpeed = 0.4;

    public FloorFeeder() {
        configureMotors();
    }

    private void configureMotors() {
        m_motor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
        // TODO: STUDENTS - Invert if needed
        // m_motor.setInverted(true);
    }

    public Command runFeederCommand() {
        return this.runEnd(
                () -> m_motor.set(kRunSpeed),
                () -> m_motor.stopMotor());
    }
}
