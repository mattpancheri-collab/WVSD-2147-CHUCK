package frc.robot.commands;

import static frc.robot.Constants.IntakeFloorConstants.kIntakeVolts;
import static frc.robot.Constants.IntakePivotConstants.*;
import static frc.robot.Constants.LauncherConstants.kShooterCloseRPS;
import static frc.robot.Constants.LauncherConstants.kShooterShotBoostVolts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FloorFeederConstants;
import frc.robot.Constants.LaunchFeederConstants;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.Launcher;

public final class LauncherFactory {
  private LauncherFactory() {}

  public static Command shootFeedVelocity(
      Launcher launcher,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder,
      IntakeGround intakeGround,
      IntakePivot intakePivot) {

    final double shooterRps = kShooterCloseRPS;

    final double floorFeederFullRps = FloorFeederConstants.kFeedInRPS;
    final double launchFeederFullRps = LaunchFeederConstants.kFeedInRPS;
    final double intakeFullVolts = kIntakeVolts;

    final double floorFeederRecoverRps = floorFeederFullRps * 0.20;
    final double intakeRecoverVolts = intakeFullVolts * 0.20;

    Command shooterCommand = Commands.run(
        () -> {
          boolean atSpeed = launcher.shooterAtSpeed();

          if (atSpeed) {
            launcher.setShooterRps(shooterRps, kShooterShotBoostVolts);
          } else {
            launcher.setShooterRps(shooterRps, 0.0);
          }

          SmartDashboard.putBoolean("LauncherFactory/Active", true);
          SmartDashboard.putBoolean("LauncherFactory/AtSpeedGate", atSpeed);
          SmartDashboard.putNumber("LauncherFactory/TargetShooterRPS", shooterRps);
          SmartDashboard.putNumber(
              "LauncherFactory/CommandedShotBoostVolts",
              atSpeed ? kShooterShotBoostVolts : 0.0);
        },
        launcher);

    Command feedCommand = Commands.run(
        () -> {
          boolean atSpeed = launcher.shooterAtSpeed();

          if (atSpeed) {
            floorFeeder.setRps(floorFeederFullRps);
            launchFeeder.setRps(launchFeederFullRps);
            intakeGround.setVoltage(intakeFullVolts);

            SmartDashboard.putString("LauncherFactory/FeedState", "FULL");
            SmartDashboard.putNumber("LauncherFactory/FloorFeederCmdRPS", floorFeederFullRps);
            SmartDashboard.putNumber("LauncherFactory/LaunchFeederCmdRPS", launchFeederFullRps);
            SmartDashboard.putNumber("LauncherFactory/IntakeCmdVolts", intakeFullVolts);
          } else {
            floorFeeder.setRps(floorFeederRecoverRps);
            launchFeeder.stop();
            intakeGround.setVoltage(intakeRecoverVolts);

            SmartDashboard.putString("LauncherFactory/FeedState", "RECOVER");
            SmartDashboard.putNumber("LauncherFactory/FloorFeederCmdRPS", floorFeederRecoverRps);
            SmartDashboard.putNumber("LauncherFactory/LaunchFeederCmdRPS", 0.0);
            SmartDashboard.putNumber("LauncherFactory/IntakeCmdVolts", intakeRecoverVolts);
          }
        },
        floorFeeder, launchFeeder, intakeGround);

    Command intakePivotOscillation =
        Commands.sequence(
                Commands.runOnce(
                    () -> intakePivot.setAngleDegrees(kIntakePivotLaunchDWNDeg), intakePivot),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> intakePivot.setAngleDegrees(kIntakePivotLaunchUpDeg), intakePivot),
                Commands.waitSeconds(1.0))
            .repeatedly();

    return shooterCommand
        .alongWith(feedCommand, intakePivotOscillation)
        .beforeStarting(() -> {
          SmartDashboard.putBoolean("LauncherFactory/Active", true);
          SmartDashboard.putBoolean("LauncherFactory/AtSpeedGate", false);
          SmartDashboard.putString("LauncherFactory/FeedState", "STARTING");
          SmartDashboard.putNumber("LauncherFactory/FloorFeederCmdRPS", 0.0);
          SmartDashboard.putNumber("LauncherFactory/LaunchFeederCmdRPS", 0.0);
          SmartDashboard.putNumber("LauncherFactory/IntakeCmdVolts", 0.0);
          SmartDashboard.putNumber("LauncherFactory/CommandedShotBoostVolts", 0.0);

          System.out.println("[LauncherFactory] shootFeedVelocity STARTING");
        })
        .finallyDo(interrupted -> {
          launcher.stop();
          floorFeeder.stop();
          launchFeeder.stop();
          intakeGround.setVoltage(0.0);

          SmartDashboard.putBoolean("LauncherFactory/Active", false);
          SmartDashboard.putBoolean("LauncherFactory/AtSpeedGate", false);
          SmartDashboard.putString("LauncherFactory/FeedState", "OFF");
          SmartDashboard.putNumber("LauncherFactory/FloorFeederCmdRPS", 0.0);
          SmartDashboard.putNumber("LauncherFactory/LaunchFeederCmdRPS", 0.0);
          SmartDashboard.putNumber("LauncherFactory/IntakeCmdVolts", 0.0);
          SmartDashboard.putNumber("LauncherFactory/CommandedShotBoostVolts", 0.0);

          System.out.println("[LauncherFactory] shootFeedVelocity ENDED interrupted=" + interrupted);
        });
  }
}