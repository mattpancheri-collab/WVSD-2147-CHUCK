package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.TestingConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Subsystems
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.Climber;

// Factories
import frc.robot.commands.IntakeFactory;
import frc.robot.commands.LauncherFactory;

public class RobotContainer {

  private final double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Swerve requests
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake =
      new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Controllers
  private final CommandXboxController driverJoystick =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final CommandXboxController operatorJoystick =
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

  private final CommandXboxController testingJoystick =
      new CommandXboxController(Constants.OperatorConstants.kTestingControllerPort);

  // Subsystems
  public final CommandSwerveDrivetrain drivetrain =
      TunerConstants.createDrivetrain();
  public final Launcher launcher = new Launcher();
  public final LaunchFeeder launchFeeder = new LaunchFeeder();
  public final FloorFeeder floorFeeder = new FloorFeeder();
  public final IntakePivot intakePivot = new IntakePivot();
  public final IntakeGround intakeGround = new IntakeGround();
  public final Climber climber = new Climber();

  public RobotContainer() {
    configureBindings();
  }

  // ---------------------------------------------------------------------------
  // HELPERS
  // ---------------------------------------------------------------------------

  /** Clamp test voltages to safe motor range */
  private static double clamp(double volts) {
    return MathUtil.clamp(volts, -12.0, 12.0);
  }

  /** Simple helper for voltage-hold buttons */
  private static Command holdVolts(
      edu.wpi.first.wpilibj2.command.Subsystem req,
      Runnable start,
      Runnable end) {
    return Commands.startEnd(start, end, req);
  }

  private void configureBindings() {

    // =========================================================================
    // DRIVER (SWERVE)
    // =========================================================================
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.registerTelemetry(logger::telemeterize);

    // =========================================================================
    // OPERATOR
    // =========================================================================
    operatorJoystick.rightBumper().whileTrue(launcher.closeShotCommand());
    operatorJoystick.leftBumper().whileTrue(launchFeeder.feederInCommand());

    operatorJoystick.x().whileTrue(intakeGround.intakeInCommand());
    operatorJoystick.b().whileTrue(intakeGround.intakeOutCommand());

    operatorJoystick.y().onTrue(intakePivot.runOnce(
        () -> intakePivot.setAngleDegrees(90.0)));
    operatorJoystick.a().onTrue(intakePivot.runOnce(
        () -> intakePivot.setAngleDegrees(0.0)));

    // =========================================================================
    // TESTING JOYSTICK (VOLTAGE ONLY)
    // =========================================================================

    final double SHOOTER_VOLTS        = clamp(kTestVoltsShooter);
    final double FLOOR_FEEDER_VOLTS   = clamp(kTestVoltsFloorFeeder);
    final double LAUNCH_FEEDER_VOLTS  = clamp(kTestVoltsLaunchFeeder);
    final double INTAKE_GROUND_VOLTS  = clamp(kTestVoltsIntakeGround);
    final double PIVOT_VOLTS          = clamp(kTestVoltsIntakePivot);

    // IntakeGround
    testingJoystick.povLeft().whileTrue(
        holdVolts(intakeGround,
            () -> intakeGround.setVoltage(+INTAKE_GROUND_VOLTS),
            () -> intakeGround.setVoltage(0.0)));

    testingJoystick.povRight().whileTrue(
        holdVolts(intakeGround,
            () -> intakeGround.setVoltage(-INTAKE_GROUND_VOLTS),
            () -> intakeGround.setVoltage(0.0)));

    // IntakePivot
    testingJoystick.povUp().whileTrue(
        holdVolts(intakePivot,
            () -> intakePivot.setVoltage(+PIVOT_VOLTS),
            () -> intakePivot.setVoltage(0.0)));

    testingJoystick.povDown().whileTrue(
        holdVolts(intakePivot,
            () -> intakePivot.setVoltage(-PIVOT_VOLTS),
            () -> intakePivot.setVoltage(0.0)));

    // FloorFeeder
    testingJoystick.leftTrigger(0.1).whileTrue(
        holdVolts(floorFeeder,
            () -> floorFeeder.setVoltage(+FLOOR_FEEDER_VOLTS),
            () -> floorFeeder.setVoltage(0.0)));

    testingJoystick.rightTrigger(0.1).whileTrue(
        holdVolts(floorFeeder,
            () -> floorFeeder.setVoltage(-FLOOR_FEEDER_VOLTS),
            () -> floorFeeder.setVoltage(0.0)));

    // LaunchFeeder
    testingJoystick.a().whileTrue(
        holdVolts(launchFeeder,
            () -> launchFeeder.setVoltage(LAUNCH_FEEDER_VOLTS),
            () -> launchFeeder.setVoltage(0.0)));

    testingJoystick.y().whileTrue(
        holdVolts(launchFeeder,
            () -> launchFeeder.setVoltage(-LAUNCH_FEEDER_VOLTS),
            () -> launchFeeder.setVoltage(0.0)));

    // Launcher
    testingJoystick.b().whileTrue(
        holdVolts(launcher,
            () -> launcher.setVoltage(SHOOTER_VOLTS),
            () -> launcher.setVoltage(0.0)));

    // =========================================================================
    // FACTORY COMMANDS
    // =========================================================================
    testingJoystick.leftBumper().whileTrue(
        IntakeFactory.deployAndIntakeChainVoltage(
            intakePivot, intakeGround, floorFeeder, launchFeeder));

    testingJoystick.rightBumper().whileTrue(
        LauncherFactory.shootFeedVoltage(
            launcher, floorFeeder, launchFeeder));
  }

  public Command getAutonomousCommand() {
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        drivetrain.applyRequest(() ->
            drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        drivetrain.applyRequest(() -> idle));
  }
}
