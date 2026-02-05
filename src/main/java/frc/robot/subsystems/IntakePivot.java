package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismIds;

public class IntakePivot extends SubsystemBase {
    private final TalonFX m_pivotMotor = new TalonFX(MechanismIds.kIntakePivotID);

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: PIVOT ANGLES (Rotations)
    // =========================================================================
    // TODO: STUDENTS - Find the correct rotations for deployed vs stowed
    private static final double kDeployedPosition = 0.25; // Example: 1/4 rotation
    private static final double kStowedPosition = 0.0;

    private final PositionVoltage m_request = new PositionVoltage(0);

    public IntakePivot() {
        configureMotors();
    }

    private void configureMotors() {
        var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        // TODO: STUDENTS - Set PID gains for the pivot
        // cfg.Slot0.kP = 5.0;
        // cfg.Slot0.kD = 0.1;

        m_pivotMotor.getConfigurator().apply(cfg);
    }

    public Command pivotToAmp() {
        return run(() -> m_pivotMotor.setControl(m_request.withPosition(kDeployedPosition)));
    }

    public Command pivotStow() {
        return run(() -> m_pivotMotor.setControl(m_request.withPosition(kStowedPosition)));
    }
}
