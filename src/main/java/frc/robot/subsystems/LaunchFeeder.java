package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismIds;

public class LaunchFeeder extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MechanismIds.kLaunchFeederID);

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: FEEDER SETTINGS
    // =========================================================================
    // TODO: STUDENTS - Adjust the speed to feed the note into the shooter
    private static final double kFeedSpeed = 0.5; // 50% power

    public LaunchFeeder() {
        configureMotors();
    }

    private void configureMotors() {
        m_motor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
        // TODO: STUDENTS - Invert if the feeder goes backwards
        // m_motor.setInverted(true);
    }

    public Command feedCommand() {
        return this.runEnd(
                () -> m_motor.set(kFeedSpeed),
                () -> m_motor.stopMotor());
    }
}
