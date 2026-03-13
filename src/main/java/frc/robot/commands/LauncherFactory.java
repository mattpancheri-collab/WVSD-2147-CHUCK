package frc.robot.commands;

import static frc.robot.Constants.IntakeFloorConstants.kIntakeVolts;
import static frc.robot.Constants.LauncherConstants.*;

import edu.wpi.first.wpilibj.Timer;
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

    final boolean[] feedEnabled = new boolean[] { false };
    final boolean[] previousFeedEnabled = new boolean[] { false };
    final Timer preHitTimer = new Timer();

    Command shooterCommand = Commands.run(() -> {
      double error = Math.abs(launcher.getShooterErrorRps());

      if (!feedEnabled[0] && error <= kShooterFeedEnableErrorRps) {
        feedEnabled[0] = true;
      }

      if (feedEnabled[0] && error >= kShooterFeedDisableErrorRps) {
        feedEnabled[0] = false;
      }

      if (feedEnabled[0] && !previousFeedEnabled[0]) {
        preHitTimer.restart();
      } else if (!feedEnabled[0] && previousFeedEnabled[0]) {
        preHitTimer.stop();
        preHitTimer.reset();
      }

      double shooterBoostVolts = 0.0;

      if (feedEnabled[0]) {
        if (preHitTimer.get() < kShooterPreHitBoostTimeSec) {
          shooterBoostVolts = kShooterPreHitBoostVolts;
        } else {
          shooterBoostVolts = kShooterShotBoostVolts;
        }
      }

      launcher.setShooterRps(shooterRps, shooterBoostVolts);

      previousFeedEnabled[0] = feedEnabled[0];
    }, launcher);

    Command feedCommand = Commands.run(() -> {
      if (feedEnabled[0]) {
        floorFeeder.setRps(floorFeederFullRps);
        launchFeeder.setRps(launchFeederFullRps);
        intakeGround.setVoltage(intakeFullVolts);
      } else {
        floorFeeder.setRps(floorFeederSlowRps);
        launchFeeder.setRps(launchFeederSlowRps);
        intakeGround.setVoltage(intakeSlowVolts);
      }
    }, floorFeeder, launchFeeder, intakeGround);

    return shooterCommand
        .alongWith(feedCommand)
        .beforeStarting(() -> {
          feedEnabled[0] = false;
          previousFeedEnabled[0] = false;
          preHitTimer.stop();
          preHitTimer.reset();
          System.out.println("[LauncherFactory] shootFeedVelocity STARTING");
        })
        .finallyDo(interrupted -> {
          preHitTimer.stop();
          launcher.stop();
          floorFeeder.stop();
          launchFeeder.stop();
          intakeGround.setVoltage(0.0);
          System.out.println("[LauncherFactory] shootFeedVelocity ENDED interrupted=" + interrupted);
        });
  }
}