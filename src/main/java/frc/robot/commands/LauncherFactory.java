package frc.robot.commands;

import static frc.robot.Constants.IntakeFloorConstants.kIntakeVolts;
import static frc.robot.Constants.IntakePivotConstants.kIntakePivotLaunchDWNDeg;
import static frc.robot.Constants.IntakePivotConstants.kIntakePivotLaunchUpDeg;
import static frc.robot.Constants.LauncherConstants.*;

import edu.wpi.first.wpilibj.Timer;
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
    final double floorFeederSlowRps = FloorFeederConstants.kFeedInRPS * 0.5;
    final double launchFeederFullRps = LaunchFeederConstants.kFeedInRPS;
    final double launchFeederSlowRps = LaunchFeederConstants.kFeedInRPS * 0.65;
    final double intakeFullVolts = kIntakeVolts;
    final double intakeSlowVolts = kIntakeVolts * 0.5;

    final boolean[] feedEnabled = new boolean[] {false};
    final boolean[] previousFeedEnabled = new boolean[] {false};
    final double[] previousShooterBoostVolts = new double[] {Double.NaN};
    final Timer preHitTimer = new Timer();

    Command shooterCommand =
        Commands.run(
            () -> {
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

              if (shooterBoostVolts != previousShooterBoostVolts[0]) {
                launcher.setShooterRps(shooterRps, shooterBoostVolts);
                previousShooterBoostVolts[0] = shooterBoostVolts;
              }

              previousFeedEnabled[0] = feedEnabled[0];

              SmartDashboard.putNumber("LauncherFactory/TargetShooterRPS", shooterRps);
              SmartDashboard.putBoolean("LauncherFactory/AtSpeedNow", launcher.shooterAtSpeed());
            },
            launcher);

    Command feedCommand =
        Commands.run(
            () -> {
              if (feedEnabled[0] != previousFeedEnabled[0]) {
                if (feedEnabled[0]) {
                  floorFeeder.setRps(floorFeederFullRps);
                  launchFeeder.setRps(launchFeederFullRps);
                  intakeGround.setVoltage(intakeFullVolts);
                } else {
                  floorFeeder.setRps(floorFeederSlowRps);
                  launchFeeder.setRps(launchFeederSlowRps);
                  intakeGround.setVoltage(intakeSlowVolts);
                }
              }
            },
            floorFeeder,
            launchFeeder,
            intakeGround);

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
        .beforeStarting(
            () -> {
              feedEnabled[0] = false;
              previousFeedEnabled[0] = false;
              previousShooterBoostVolts[0] = Double.NaN;
              preHitTimer.stop();
              preHitTimer.reset();
              intakePivot.setAngleDegrees(kIntakePivotLaunchUpDeg);

              launcher.setShooterRps(shooterRps, 0.0);
              floorFeeder.setRps(floorFeederSlowRps);
              launchFeeder.setRps(launchFeederSlowRps);
              intakeGround.setVoltage(intakeSlowVolts);

              SmartDashboard.putBoolean("LauncherFactory/AtSpeedNow", false);
              SmartDashboard.putNumber("LauncherFactory/LaunchFeederCmdRPS", 0.0);
            })
            .finallyDo(interrupted -> {
              preHitTimer.stop();
              launcher.stop();
              floorFeeder.stop();
              launchFeeder.stop();
              intakeGround.stop();   // was setVoltage(0.0)
              intakePivot.stop();
              SmartDashboard.putBoolean("LauncherFactory/AtSpeedNow", false);
          });
  }
}