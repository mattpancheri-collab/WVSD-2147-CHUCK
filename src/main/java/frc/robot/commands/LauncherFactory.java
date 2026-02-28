package frc.robot.commands;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.Launcher;

import static frc.robot.Constants.ShootingConstants.*;

public final class LauncherFactory {
  private LauncherFactory() {
  }

  public static Command shootFeedVoltage(
      Launcher launcher,
      FloorFeeder floorFeeder,
      LaunchFeeder launchFeeder) {

    // DO NOT apply polarity here.
    // Polarity is applied inside Launcher.setVoltage() so everything is consistent.
    final double shooterVolts = clamp(kShooterVolts);

    // Feeders (usually no polarity needed; flip in subsystem if required)
    final double floorFeederVolts = clamp(kFloorFeederVolts);
    final double launchFeederVolts = clamp(kLaunchFeederVolts);

    Command spinUpShooter = Commands.startEnd(
        () -> launcher.setVoltage(shooterVolts),
        () -> launcher.setVoltage(0.0),
        launcher);

    Command runFeeders = Commands.startEnd(
        () -> {
          floorFeeder.setVoltage(floorFeederVolts);
          launchFeeder.setVoltage(launchFeederVolts);
        },
        () -> {
          floorFeeder.setVoltage(0.0);
          launchFeeder.setVoltage(0.0);
        },
        floorFeeder,
        launchFeeder);

    return spinUpShooter.alongWith(
        Commands.waitSeconds(kShooterSpinUpSeconds).andThen(runFeeders));
  }

  private static double clamp(double volts) {
    return MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
  }
}
