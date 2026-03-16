package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LaunchFeeder;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.IntakeFloorConstants;

public final class IntakeFactory {
  private IntakeFactory() {
  }

  public static Command deployAndIntakeChainVoltage(
      IntakePivot intakePivot,
      IntakeGround intakeGround,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder) {

    final double intakeVolts = IntakeFloorConstants.kIntakeVolts;

    Command intakeCmd = Commands.startEnd(
        () -> intakeGround.setVoltage(+intakeVolts),
        () -> intakeGround.setVoltage(0.0),
        intakeGround);

    return intakeCmd;
  }

  public static Command intakeOnlyCommand(IntakeGround intakeGround, IntakePivot intakePivot) {
    return Commands.parallel(
        intakePivot.setAngleCommand(IntakePivotConstants.kIntakeAngleDeg),
        Commands.startEnd(
            () -> intakeGround.setVoltage(IntakeFloorConstants.kIntakeVolts),
            intakeGround::stop,
            intakeGround
        )
    );
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

    final double p = MathUtil.clamp(percent, 0.0, 1.0);

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

}
