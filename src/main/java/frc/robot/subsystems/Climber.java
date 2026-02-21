package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.BusConstants;
import frc.robot.Constants.CANConstants;

public class Climber extends SubsystemBase {

  private final TalonFX motor = new TalonFX(CANConstants.kClimberID, BusConstants.kDefaultBus);

  public Climber() {
    configureMotor();
    stop();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimitEnable = ClimberConstants.kEnableStatorLimit;
    currentLimits.StatorCurrentLimit = ClimberConstants.kStatorLimitAmps;

    motor.getConfigurator().apply(config);
  }

  // ---------------------------------------------------------------------------
  // Control API
  // ---------------------------------------------------------------------------

  public void stop() {
    motor.set(0.0);
  }

  public void climb() {
    motor.set(ClimberConstants.kClimbPower);
  }

  public void reverseClimb() {
    motor.set(ClimberConstants.kClimbReversePower);
  }

  // ---------------------------------------------------------------------------
  // Commands (no finallyDo)
  // ---------------------------------------------------------------------------

  public Command climbCommand() {
    return Commands.startEnd(this::climb, this::stop, this);
  }

  public Command reverseClimbCommand() {
    return Commands.startEnd(this::reverseClimb, this::stop, this);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
