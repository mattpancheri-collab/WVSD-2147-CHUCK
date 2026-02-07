package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.Launcher;

/**
 * LauncherFactory builds launcher/shooter-related multi-subsystem commands.
 *
 * Rules:
 * - Static methods only
 * - Do NOT store subsystems as fields
 * - Do NOT read joysticks here
 * - Only build and return Commands
 */
public final class LauncherFactory {
  private LauncherFactory() {
  }

  /**
   * While held:
   * - Spin launcher flywheels at percent of max RPS
   * - Run floor feeder at percent of max RPS
   * - Run launch feeder at percent of max RPS
   *
   * On release/end:
   * - Stop floor feeder
   * - Stop launch feeder
   * - Stop launcher
   *
   * @param launcher           shooter flywheels subsystem
   * @param floorFeeder        intake/floor feeder subsystem
   * @param launchFeeder       launcher feeder subsystem
   * @param percent            0..1 (use 0.90 for 90%)
   * @param launcherMaxRps     safe cap for shooter (RPS)
   * @param floorFeederMaxRps  safe cap for floor feeder (RPS)
   * @param launchFeederMaxRps safe cap for launch feeder (RPS)
   */
  public static Command shootFeedPercent(
      Launcher launcher,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder,
      double percent,
      double launcherMaxRps,
      double floorFeederMaxRps,
      double launchFeederMaxRps) {

    final double p = clamp(percent, 0.0, 1.0);

    final double launcherRps = p * launcherMaxRps;
    final double floorFeederRps = p * floorFeederMaxRps;
    final double launchFeederRps = p * launchFeederMaxRps;

    return Commands.parallel(
        launcher.runShooterRpsCommand(launcherRps),
        floorFeeder.feederCommand(floorFeederRps),
        launchFeeder.feederCommand(launchFeederRps))
        .finallyDo(
            interrupted -> {
              CommandScheduler.getInstance().schedule(floorFeeder.stopCommand());
              CommandScheduler.getInstance().schedule(launchFeeder.stopCommand());
              CommandScheduler.getInstance().schedule(launcher.runShooterRpsCommand(0.0));
            });
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
