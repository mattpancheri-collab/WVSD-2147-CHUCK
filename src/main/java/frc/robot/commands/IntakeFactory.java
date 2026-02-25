package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LaunchFeeder;

import static frc.robot.Constants.TestingConstants.*;

public final class IntakeFactory {
  private IntakeFactory() {
  }

  public static Command deployAndIntakeChainVoltage(
      IntakePivot intakePivot,
      IntakeGround intakeGround,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder) {

    final double intakeVolts = clampVolts(kTestVoltsIntakeGround);
    final double floorVolts = clampVolts(kTestVoltsFloorFeeder);
    final double launchVolts = clampVolts(kTestVoltsLaunchFeeder);

    // Pivot: deploy while held, stow on release
    Command pivotCmd = Commands.startEnd(
        () -> intakePivot.setAngleDegrees(frc.robot.Constants.IntakePivotConstants.kDeployAngleDeg),
        () -> intakePivot.setAngleDegrees(frc.robot.Constants.IntakePivotConstants.kMinAngleDeg),
        intakePivot);

    // Intake roller
    Command intakeCmd = Commands.startEnd(
        () -> intakeGround.setVoltage(+intakeVolts),
        () -> intakeGround.setVoltage(0.0),
        intakeGround);

    // Floor feeder
    Command floorCmd = Commands.startEnd(
        () -> floorFeeder.setVoltage(+floorVolts),
        () -> floorFeeder.setVoltage(0.0),
        floorFeeder);

    // Launch feeder
    Command launchCmd = Commands.startEnd(
        () -> launchFeeder.setVoltage(+launchVolts),
        () -> launchFeeder.setVoltage(0.0),
        launchFeeder);

    // Parallel group already stops on end because each is startEnd.
    // (No finallyDo needed, so this works across WPILib versions.)
    return Commands.parallel(pivotCmd, intakeCmd, floorCmd, launchCmd);
  }

  // ---------------------------------------------------------------------------
  // Your existing percent/RPS version (kept)
  // ---------------------------------------------------------------------------
  public static Command deployAndIntakeChainPercent(
      IntakePivot intakePivot,
      IntakeGround intakeGround,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder,
      double deployDeg,
      double stowDeg,
      double percent,
      double intakeMaxRps,
      double floorMaxRps,
      double launchMaxRps) {

    final double p = clamp(percent, 0.0, 1.0);

    final double intakeRps = p * intakeMaxRps;
    final double floorRps = p * floorMaxRps;
    final double launchRps = p * launchMaxRps;

    Command launchFeederAutoStop = Commands.run(
        () -> {
          if (launchFeeder.hasBall()) {
            launchFeeder.stop();
          } else {
            launchFeeder.setRps(launchRps);
          }
        },
        launchFeeder).finallyDo(i -> launchFeeder.stop()); // <-- if this errors, tell me and I'll swap it too

    return Commands.parallel(
        intakePivot.setAngleCommand(deployDeg),
        intakeGround.intakeCommand(intakeRps),
        floorFeeder.feederCommand(floorRps),
        launchFeederAutoStop).andThen(intakePivot.setAngleCommand(stowDeg));
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private static double clampVolts(double volts) {
    return clamp(volts, -12.0, 12.0);
  }
}
