package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismIds;

public class Launcher extends SubsystemBase {
    // =========================================================================
    // HARDWARE DECLARATIONS
    // =========================================================================
    // We use TalonFX (Kraken/Falcon) motors for the launcher.
    private final TalonFX m_leftMotor = new TalonFX(MechanismIds.kLauncherLeftID);
    private final TalonFX m_rightMotor = new TalonFX(MechanismIds.kLauncherRightID);

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: SPEED SETTINGS
    // =========================================================================
    // TODO: STUDENTS - Tune these RPM values for your specific shot
    private static final double kTargetRPM = 3000.0;
    private static final double kEjectRPM = 1000.0;

    // Request to run at a specific velocity (closed-loop)
    private final VelocityVoltage m_request = new VelocityVoltage(0);

    /** Creates a new Launcher. */
    public Launcher() {
        // TODO: STUDENTS - Configure Motor Inverts if needed
        // m_rightMotor.setInverted(true);

        configureMotors();
    }

    private void configureMotors() {
        // Apply factory defaults to ensure we start in a known state
        m_leftMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
        m_rightMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

        // TODO: STUDENTS - Add PID configuration here if the default isn't enough
        // var pidConfig = new Slot0Configs();
        // pidConfig.kP = 0.11;
        // m_leftMotor.getConfigurator().apply(pidConfig);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Useful for logging checking motor temperatures or speeds
    }

    /**
     * Command to run the launcher at a set speed.
     */
    public Command runLauncherCommand() {
        // Inline command: Runs the motor at target RPM, stops when command ends.
        return this.runEnd(
                () -> {
                    m_leftMotor.setControl(m_request.withVelocity(kTargetRPM));
                    m_rightMotor.setControl(m_request.withVelocity(kTargetRPM));
                },
                () -> {
                    m_leftMotor.stopMotor();
                    m_rightMotor.stopMotor();
                });
    }
}
