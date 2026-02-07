package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakeFloor;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LaunchFeeder;

/**
 * IntakeFactory builds multi-subsystem intake commands.
 *
 * Rules:
 * - Static methods only
 * - Do NOT store subsystems as fields
 * - Do NOT read joysticks here
 * - Only build and return Commands
 */
public final class IntakeFactory {
  private IntakeFactory() {
  }

  /**
   * Deploy intake + run intake chain at a PERCENT of each mechanism's max RPS.
   *
   * Behavior while held:
   * - IntakePivot holds 90 deg (position PID)
   * - IntakeFloor runs forward at 90% (velocity PID)
   * - FloorFeeder runs forward at 90% (velocity PID)
   * - LaunchFeeder runs forward at 90% UNTIL CANrange sees a ball, then it STOPS
   * (intakeFloor + floorFeeder keep running regardless)
   *
   * Behavior on release/end:
   * - IntakeFloor stops
   * - FloorFeeder stops
   * - LaunchFeeder stops
   * - Pivot optionally stows to stowDeg (recommended; pass 0.0 if you want stow)
   *
   * @param intakePivot        pivot subsystem
   * @param intakeFloor        floor intake roller subsystem
   * @param floorFeeder        floor feeder subsystem
   * @param launchFeeder       launcher feeder (CANrange beam break)
   * @param deployDeg          pivot angle while held (use 90.0)
   * @param stowDeg            pivot angle on release (use 0.0 if you want it to
   *                           go back up)
   * @param percent            0.0 to 1.0 (use 0.90 for 90%)
   * @param intakeFloorMaxRps  max RPS cap used for percent -> RPS conversion
   * @param floorFeederMaxRps  max RPS cap used for percent -> RPS conversion
   * @param launchFeederMaxRps max RPS cap used for percent -> RPS conversion
   */
  public static Command deployAndIntakeChainPercent(
      IntakePivot intakePivot,
      IntakeFloor intakeFloor,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder,
      double deployDeg,
      double stowDeg,
      double percent,
      double intakeFloorMaxRps,
      double floorFeederMaxRps,
      double launchFeederMaxRps) {

    final double p = clamp(percent, 0.0, 1.0);

    final double intakeFloorRps = p * intakeFloorMaxRps;
    final double floorFeederRps = p * floorFeederMaxRps;
    final double launchFeederRps = p * launchFeederMaxRps;

    // This command ONLY controls the launch feeder:
    // - If ball detected => stop launch feeder
    // - Else => run at launchFeederRps
    // It never ends on its own; it ends when the overall command ends (button
    // released).
    Command launchFeederAutoStop = Commands.run(
        () -> {
          if (launchFeeder.hasBall()) {
            launchFeeder.stop();
          } else {
            launchFeeder.setRps(launchFeederRps);
          }
        },
        launchFeeder)
        .finallyDo(interrupted -> launchFeeder.stop());

    // Run everything in parallel; this whole command runs until the button is
    // released.
    return Commands.parallel(
        // Hold pivot down
        intakePivot.setAngleCommand(deployDeg),

        // Keep these running the whole time
        intakeFloor.intakeCommand(intakeFloorRps),
        floorFeeder.feederCommand(floorFeederRps),

        // Launch feeder with ball-detect auto stop (only this one stops on ball)
        launchFeederAutoStop)
        .finallyDo(
            interrupted -> {
              // Ensure motors are stopped when button released
              CommandScheduler.getInstance().schedule(intakeFloor.stopCommand());
              CommandScheduler.getInstance().schedule(floorFeeder.stopCommand());
              CommandScheduler.getInstance().schedule(launchFeeder.stopCommand());

              // Recommended: stow pivot back up when released
              CommandScheduler.getInstance().schedule(intakePivot.setAngleCommand(stowDeg));
            });
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
