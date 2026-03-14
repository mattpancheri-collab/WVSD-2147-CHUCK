package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.BusConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivot extends SubsystemBase {

    private enum ControlMode {
        POSITION,
        VELOCITY,
        VOLTAGE,
        STOPPED
    }

    private final TalonFX pivotMotor =
        new TalonFX(CANConstants.kPivotID, BusConstants.kDefaultBus);

    private final TalonFX pivotFollower =
        new TalonFX(CANConstants.kPivotFollowerID, BusConstants.kDefaultBus);

    private final Follower pivotFollowerRequest =
        new Follower(CANConstants.kPivotID, MotorAlignmentValue.Opposed);

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    private final SlewRateLimiter rpsLimiter =
        new SlewRateLimiter(IntakePivotConstants.kRampRPSPerSec);

    private ControlMode m_controlMode = ControlMode.STOPPED;
    private double targetRps = 0.0;
    private double targetDeg = IntakePivotConstants.kIdleAngleDeg;
    private double voltageDemand = 0.0;

    public IntakePivot() {
        configureLeader();
        configureFollower();

        // Latch follower once at startup
        pivotFollower.setControl(pivotFollowerRequest);

        // Start at idle
        setAngleDegrees(IntakePivotConstants.kIdleAngleDeg);
    }

    private void configureLeader() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = IntakePivotConstants.kGearRatio;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = IntakePivotConstants.kP;
        slot0.kI = IntakePivotConstants.kI;
        slot0.kD = IntakePivotConstants.kD;
        slot0.kG = IntakePivotConstants.kG;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        currentLimits.StatorCurrentLimitEnable = IntakePivotConstants.kEnableStatorLimit;
        currentLimits.StatorCurrentLimit = IntakePivotConstants.kStatorLimitAmpsUp;

        pivotMotor.getConfigurator().apply(config);
        pivotMotor.getConfigurator().apply(currentLimits);
    }

    private void configureFollower() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotFollower.getConfigurator().apply(config);
        pivotFollower.getConfigurator().apply(currentLimits);
    }

    public void setAngleDegrees(double deg) {
        System.out.println("[IntakePivot] setAngleDegrees TARGET: " + deg);

        targetRps = 0.0;
        rpsLimiter.reset(0.0);
        voltageDemand = 0.0;

        double currentPos = getPositionDeg();
        double newTarget = MathUtil.clamp(deg, -20.0, 100.0);

        double newLimit = (newTarget < currentPos)
            ? IntakePivotConstants.kStatorLimitAmpsUp
            : IntakePivotConstants.kStatorLimitAmpsDown;

        if (newLimit != currentLimits.StatorCurrentLimit) {
            currentLimits.StatorCurrentLimit = newLimit;
            pivotMotor.getConfigurator().apply(currentLimits);
            pivotFollower.getConfigurator().apply(currentLimits);
            System.out.println("[IntakePivot] Swapping current limit to: " + newLimit + " A");
        }

        targetDeg = newTarget;
        double pivotRotations = targetDeg / 360.0;
        pivotMotor.setControl(positionRequest.withPosition(pivotRotations));
        m_controlMode = ControlMode.POSITION;
    }

    public void setRps(double rps) {
        voltageDemand = 0.0;
        double maxRps = IntakePivotConstants.kMaxVelocityRPS;
        targetRps = MathUtil.clamp(rps, -maxRps, maxRps);
        m_controlMode = ControlMode.VELOCITY;
    }

    public void setVoltage(double volts) {
        if (volts != 0.0) {
            System.out.println("[IntakePivot] setVoltage: " + volts);
        }

        targetRps = 0.0;
        rpsLimiter.reset(0.0);
        voltageDemand = MathUtil.clamp(volts, -12.0, 12.0);
        m_controlMode = ControlMode.VOLTAGE;
    }

    public void stop() {
        targetRps = 0.0;
        rpsLimiter.reset(0.0);
        voltageDemand = 0.0;
        pivotMotor.setVoltage(0.0);
        pivotFollower.setVoltage(0.0);
        m_controlMode = ControlMode.STOPPED;
    }

    public void deploy() {
        setAngleDegrees(IntakePivotConstants.kIntakeAngleDeg);
    }

    public void stow() {
        setAngleDegrees(IntakePivotConstants.kIdleAngleDeg);
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public double getPositionDeg() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean isVoltageOverride() {
        return m_controlMode == ControlMode.VOLTAGE;
    }

    public double getVoltageDemand() {
        return voltageDemand;
    }

    @Override
    public void periodic() {
        // Re-latch follower every loop in case of reset/brownout
        pivotFollower.setControl(pivotFollowerRequest);

        switch (m_controlMode) {
            case POSITION:
                double pivotRotations = targetDeg / 360.0;
                pivotMotor.setControl(positionRequest.withPosition(pivotRotations));

                if (Timer.getFPGATimestamp() % 0.5 < 0.02) {
                    System.out.println("[IntakePivot] POS: " + getPositionDeg() + " -> " + targetDeg);
                }
                break;

            case VELOCITY:
                double limitedRps = rpsLimiter.calculate(targetRps);
                pivotMotor.setControl(velocityRequest.withVelocity(limitedRps));
                break;

            case VOLTAGE:
                pivotMotor.setVoltage(voltageDemand);
                break;

            case STOPPED:
            default:
                break;
        }
    }

    public Command deployWhileHeldStowOnReleaseCommand() {
        return Commands.startEnd(this::deploy, this::stow, this);
    }

    public Command pivotDeployCommand() {
        return run(this::deploy);
    }

    public Command pivotStowCommand() {
        return run(this::stow);
    }

    public Command setAngleCommand(double deg) {
        return run(() -> setAngleDegrees(deg));
    }

    public Command pivotVoltageCommand(double volts) {
        return Commands.startEnd(() -> setVoltage(volts), () -> setVoltage(0.0), this);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}