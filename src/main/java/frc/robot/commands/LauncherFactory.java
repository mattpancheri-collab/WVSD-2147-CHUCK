package frc.robot.commands;

import static frc.robot.Constants.IntakeFloorConstants.kIntakeVolts;
import static frc.robot.Constants.LauncherConstants.kShooterCloseRPS;
import static frc.robot.Constants.LauncherConstants.kShooterShotBoostVolts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FloorFeederConstants;
import frc.robot.Constants.LaunchFeederConstants;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.Launcher;

public final class LauncherFactory {
  private LauncherFactory() {}

  public static Command shootFeedVelocity(
      Launcher launcher,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder,
      IntakeGround intakeGround) {

    final double shooterRps = kShooterCloseRPS;

    final double floorFeederFullRps = FloorFeederConstants.kFeedInRPS;
    final double floorFeederSlowRps = FloorFeederConstants.kFeedInRPS * 0.25;

    final double launchFeederFullRps = LaunchFeederConstants.kFeedInRPS;
    final double launchFeederSlowRps = LaunchFeederConstants.kFeedInRPS * 0.25;

    final double intakeFullVolts = kIntakeVolts;
    final double intakeSlowVolts = kIntakeVolts * 0.25;

    Command shooterCommand = Commands.run(
        () -> {
          if (launcher.shooterAtSpeed()) {
            launcher.setShooterRps(shooterRps, kShooterShotBoostVolts);
          } else {
            launcher.setShooterRps(shooterRps, 0.0);
          }
        },
        launcher);

    Command feedCommand = Commands.run(
        () -> {
          if (launcher.shooterAtSpeed()) {
            floorFeeder.setRps(floorFeederFullRps);
            launchFeeder.setRps(launchFeederFullRps);
            intakeGround.setVoltage(intakeFullVolts);
          } else {
            floorFeeder.setRps(floorFeederSlowRps);
            launchFeeder.setRps(launchFeederSlowRps);
            intakeGround.setVoltage(intakeSlowVolts);
          }
        },
        floorFeeder, launchFeeder, intakeGround);

    return shooterCommand
        .alongWith(feedCommand)
        .beforeStarting(() -> System.out.println("[LauncherFactory] shootFeedVelocity STARTING"))
        .finallyDo(interrupted -> {
          launcher.stop();
          floorFeeder.stop();
          launchFeeder.stop();
          intakeGround.setVoltage(0.0);
          System.out.println("[LauncherFactory] shootFeedVelocity ENDED interrupted=" + interrupted);
        });
  }
}